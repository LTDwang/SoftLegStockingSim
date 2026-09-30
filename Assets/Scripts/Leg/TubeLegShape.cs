//通用管状腿形：中心线 + 基础半径图 r0[行, 列]
//软组织层（凹陷、鼓起、以后的晃动）、碰撞与显示都建立在这张网格上。
//以后新增"程序化大腿""导入模型烘焙"时只需增加数据来源，其它部分不用改。
//中心线目前是局部 +Z 方向的直线，换成样条时只需修改本类的坐标转换。

using UnityEngine;

public enum LegRadiusSource
{
    Cylinder,//圆柱：半径恒定
    ProceduralThigh,//程序化大腿：上下半径渐变 + 椭圆截面
}

public class TubeLegShape : MonoBehaviour
{
    [Header("Centerline")]

    [SerializeField]
    [Tooltip("中心线沿局部 +Z 方向的长度")]
    private float length = 3f;

    [Header("Grid")]

    [SerializeField]
    [Tooltip("沿腿长度方向的采样行数（运行中修改不生效）")]
    private int axialResolution = 96;

    [SerializeField]
    [Tooltip("绕腿一圈的采样列数（运行中修改不生效）")]
    private int radialResolution = 64;

    [Header("Radius Source")]

    [SerializeField]
    private LegRadiusSource source = LegRadiusSource.Cylinder;

    [SerializeField]
    [Tooltip("圆柱半径（数据来源为 Cylinder 时使用）")]
    private float cylinderRadius = 1f;

    [Header("Procedural Thigh")]

    [SerializeField]
    [Tooltip("z = 0 端（膝盖侧）的半径")]
    private float bottomRadius = 0.8f;

    [SerializeField]
    [Tooltip("z = length 端（髋部侧）的半径")]
    private float topRadius = 1.05f;

    [SerializeField]
    [Tooltip("沿长度 0→1 从下端半径过渡到上端半径的插值曲线，可调出中段肌肉的隆起")]
    private AnimationCurve radiusProfile = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

    [SerializeField]
    [Range(0.5f, 1f)]
    [Tooltip("截面扁度：短轴 / 长轴，1 为正圆")]
    private float flatten = 0.85f;

    [SerializeField]
    [Tooltip("截面短轴的朝向（度），用于表现大腿内侧偏平")]
    private float flattenAngle = 0f;

    // 以下为运行时缓存，不参与序列化（编辑器重载脚本时不会残留不一致的状态）

    // 基础半径 r0，按 (行, 列) 存储
    [System.NonSerialized]
    private float[] baseRadius;

    // 每行平均半径，用于把绕腿方向的格子换算成弧长
    [System.NonSerialized]
    private float[] rowMeanRadius;

    [System.NonSerialized]
    private float maxBaseRadius;

    [System.NonSerialized]
    private int builtAxial;

    [System.NonSerialized]
    private int builtRadial;

    [System.NonSerialized]
    private bool built;

    public float Length => length;

    public int AxialResolution { get { EnsureBuilt(); return builtAxial; } }

    public int RadialResolution { get { EnsureBuilt(); return builtRadial; } }

    public float MaxBaseRadius { get { EnsureBuilt(); return maxBaseRadius; } }

    public Vector3 AxisWorld => transform.TransformDirection(Vector3.forward).normalized;

    private void OnValidate()
    {
        built = false;
    }

    public void EnsureBuilt()
    {
        if (!built || !HasValidGrid())
        {
            Build();
        }
    }

    private bool HasValidGrid()
    {
        return baseRadius != null && builtAxial >= 2 && builtRadial >= 3 && baseRadius.Length == builtAxial * builtRadial;
    }

    private void Build()
    {
        // 运行中网格尺寸已被软组织层按旧尺寸分配，只允许刷新半径值
        bool keepSize = Application.isPlaying && HasValidGrid();

        if (!keepSize)
        {
            builtAxial = Mathf.Max(2, axialResolution);
            builtRadial = Mathf.Max(3, radialResolution);
            baseRadius = new float[builtAxial * builtRadial];
            rowMeanRadius = new float[builtAxial];
        }

        maxBaseRadius = 0f;

        for (int i = 0; i < builtAxial; i++)
        {
            float sum = 0f;

            for (int j = 0; j < builtRadial; j++)
            {
                float r = ComputeBaseRadius(i, j);
                baseRadius[GetFieldIndex(i, j)] = r;
                sum += r;
                maxBaseRadius = Mathf.Max(maxBaseRadius, r);
            }

            rowMeanRadius[i] = sum / builtRadial;
        }

        built = true;
    }

    private float ComputeBaseRadius(int axialIndex, int radialIndex)
    {
        switch (source)
        {
            case LegRadiusSource.ProceduralThigh:
                return ProceduralThighRadius(axialIndex, radialIndex);

            case LegRadiusSource.Cylinder:
            default:
                return cylinderRadius;
        }
    }

    // 程序化大腿：轮廓半径沿长度渐变，截面为面积不变的椭圆（长半轴 a = R/√扁度，短半轴 b = R·√扁度）
    private float ProceduralThighRadius(int axialIndex, int radialIndex)
    {
        float t = (float)axialIndex / (builtAxial - 1);
        float profile = radiusProfile != null && radiusProfile.length > 0 ? radiusProfile.Evaluate(t) : t;
        float r = Mathf.LerpUnclamped(bottomRadius, topRadius, profile);

        float squash = Mathf.Sqrt(Mathf.Clamp(flatten, 0.01f, 1f));
        float a = r / squash;
        float b = r * squash;

        float phi = 2f * Mathf.PI * radialIndex / builtRadial - flattenAngle * Mathf.Deg2Rad;
        float bCos = b * Mathf.Cos(phi);
        float aSin = a * Mathf.Sin(phi);

        return a * b / Mathf.Sqrt(bCos * bCos + aSin * aSin);
    }

    // =========================
    // Grid
    // =========================

    public int GetFieldIndex(int axialIndex, int radialIndex)
    {
        return axialIndex * builtRadial + radialIndex;
    }

    public float GetBaseRadius(int axialIndex, int radialIndex)
    {
        EnsureBuilt();
        return baseRadius[GetFieldIndex(axialIndex, radialIndex)];
    }

    public float RowMeanRadius(int axialIndex)
    {
        EnsureBuilt();
        return rowMeanRadius[axialIndex];
    }

    public void GetBilinear(float localZ, float angle,
        out int i0, out int i1, out int j0, out int j1, out float fz, out float fa)
    {
        EnsureBuilt();

        float u = Mathf.Clamp01(localZ / Mathf.Max(length, 0.000001f)) * (builtAxial - 1);
        i0 = Mathf.Min((int)u, builtAxial - 2);
        i1 = i0 + 1;
        fz = u - i0;

        float v = Mathf.Repeat(angle / (2f * Mathf.PI), 1f) * builtRadial;
        int j = (int)v;
        fa = v - j;
        j0 = j % builtRadial;
        j1 = (j0 + 1) % builtRadial;
    }

    // 局部 (z, θ) 处的基础半径（双线性插值）
    public float SampleBaseRadius(float localZ, float angle)
    {
        GetBilinear(localZ, angle, out int i0, out int i1, out int j0, out int j1, out float fz, out float fa);

        return SampleBaseRadius(i0, i1, j0, j1, fz, fa);
    }

    // 已算好双线性位置时直接取值，供碰撞等热点路径复用同一次查找
    public float SampleBaseRadius(int i0, int i1, int j0, int j1, float fz, float fa)
    {
        return
            baseRadius[GetFieldIndex(i0, j0)] * (1f - fz) * (1f - fa)
            + baseRadius[GetFieldIndex(i0, j1)] * (1f - fz) * fa
            + baseRadius[GetFieldIndex(i1, j0)] * fz * (1f - fa)
            + baseRadius[GetFieldIndex(i1, j1)] * fz * fa;
    }

    // 基础半径沿腿方向、绕腿方向的变化率（中心差分），圆柱时均为 0
    public void SampleBaseRadiusGradient(float localZ, float angle, out float dRdz, out float dRdAngle)
    {
        EnsureBuilt();

        float hz = 0.5f * length / (builtAxial - 1);
        float ha = Mathf.PI / builtRadial;

        dRdz = (SampleBaseRadius(localZ + hz, angle) - SampleBaseRadius(localZ - hz, angle)) / (2f * hz);
        dRdAngle = (SampleBaseRadius(localZ, angle + ha) - SampleBaseRadius(localZ, angle - ha)) / (2f * ha);
    }

    // =========================
    // Coordinates
    // =========================

    // 世界坐标 → 管坐标：沿中心线位置 z、绕中心线角度 θ、到中心线距离 ρ
    public void WorldToTube(Vector3 worldPosition, out float localZ, out float angle, out float rho)
    {
        Vector3 local = transform.InverseTransformPoint(worldPosition);

        localZ = local.z;
        rho = Mathf.Sqrt(local.x * local.x + local.y * local.y);
        angle = rho < 0.00000001f ? 0f : Mathf.Atan2(local.y, local.x);
    }

    public Vector3 TubeToLocal(float localZ, float angle, float rho)
    {
        return new Vector3(Mathf.Cos(angle) * rho, Mathf.Sin(angle) * rho, localZ);
    }

    public Vector3 TubeToWorld(float localZ, float angle, float rho)
    {
        return transform.TransformPoint(TubeToLocal(localZ, angle, rho));
    }

    public Vector3 RadialWorld(float angle)
    {
        return transform.TransformDirection(new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0f)).normalized;
    }

    // 基础表面（不含软组织偏移）在 (z, θ) 处的坐标系
    public LegSurfaceFrame GetBaseSurfaceFrame(float localZ, float angle)
    {
        float r = SampleBaseRadius(localZ, angle);
        SampleBaseRadiusGradient(localZ, angle, out float dRdz, out float dRdAngle);

        Vector3 radialLocal = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0f);
        Vector3 aroundLocal = new Vector3(-Mathf.Sin(angle), Mathf.Cos(angle), 0f);

        // 曲面 ρ = r(z, θ) 的外法线：径向减去半径随 θ、z 的变化
        Vector3 normalLocal = radialLocal - aroundLocal * (dRdAngle / Mathf.Max(r, 0.000001f)) - Vector3.forward * dRdz;

        Vector3 normal = transform.TransformDirection(normalLocal).normalized;
        Vector3 axis = AxisWorld;
        Vector3 axialTangent = (axis - normal * Vector3.Dot(axis, normal)).normalized;
        Vector3 circumferentialTangent = Vector3.Cross(axialTangent, normal).normalized;

        return new LegSurfaceFrame
        {
            surfacePoint = TubeToWorld(localZ, angle, r),
            normal = normal,
            axialTangent = axialTangent,
            circumferentialTangent = circumferentialTangent
        };
    }
}
