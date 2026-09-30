//软组织层：丝袜压力使腿表面凹陷，凹陷边缘有软肉鼓起（勒肉）
//表面半径 r(z,θ) = r0(z,θ) + offset(z,θ)，r0 来自 TubeLegShape，offset = -凹陷 + 鼓起
//碰撞与外观（SoftLegVisual）共用同一张偏移场，丝袜会陷进勒痕里

using UnityEngine;

[RequireComponent(typeof(TubeLegShape))]
public class SoftTissueLegCollision : LegCollisionModel
{
    [Header("Soft Tissue")]

    [SerializeField]
    [Tooltip("柔软度：目标凹陷深度 = 柔软度 × 接触压力")]
    private float compliance = 15f;

    [SerializeField]
    [Tooltip("最大凹陷深度")]
    private float maxIndentation = 0.08f;

    [SerializeField]
    [Tooltip("软组织响应时间（秒），越小凹陷跟随越快")]
    private float responseTime = 0.15f;

    [SerializeField]
    [Range(0, 8)]
    [Tooltip("凹陷平滑次数")]
    private int blurPasses = 2;

    [Header("Bulge")]

    [SerializeField]
    [Tooltip("凹陷边缘鼓起强度")]
    private float bulgeStrength = 3f;

    [SerializeField]
    [Range(1, 16)]
    [Tooltip("鼓起向外扩散的范围（模糊次数）")]
    private int bulgeSpread = 8;

    private const float MinPressureWeight = 0.0001f;

    private TubeLegShape shape;

    // 从 TubeLegShape 复制的网格尺寸（不序列化）
    private int axialResolution;
    private int radialResolution;

    // 本帧累计的接触压力（推出距离 δ 的加权和）
    private float[] pressureSum;
    private float[] pressureWeight;

    // 本帧每格的压力（没被丝袜覆盖的格子为 0）
    private float[] pressure;

    // 丝袜粒子在压力场网格上的坐标（行 u、列 v），上报压力时复用
    private float[] particleU;
    private float[] particleV;

    // 当前凹陷深度（正数 = 向内）
    private float[] depth;

    // 最终径向偏移（负数 = 凹陷，正数 = 鼓起）
    private float[] offset;

    private float[] blurTemp;
    private float[] wideDepth;

    private float currentMaxIndentation;
    private float currentMaxBulge;

    public TubeLegShape Shape => shape != null ? shape : shape = GetComponent<TubeLegShape>();

    public int AxialResolution => axialResolution;

    public int RadialResolution => radialResolution;

    public float CurrentMaxIndentation => currentMaxIndentation;

    public float CurrentMaxBulge => currentMaxBulge;

    private void Awake()
    {
        shape = GetComponent<TubeLegShape>();
        shape.EnsureBuilt();

        axialResolution = shape.AxialResolution;
        radialResolution = shape.RadialResolution;

        int count = axialResolution * radialResolution;
        pressureSum = new float[count];
        pressureWeight = new float[count];
        pressure = new float[count];
        depth = new float[count];
        offset = new float[count];
        blurTemp = new float[count];
        wideDepth = new float[count];
    }

    public int GetFieldIndex(int axialIndex, int radialIndex)
    {
        return axialIndex * radialResolution + radialIndex;
    }

    public float GetRadialOffset(int axialIndex, int radialIndex)
    {
        return offset[GetFieldIndex(axialIndex, radialIndex)];
    }

    // 局部 (z, θ) 处的软组织偏移（双线性插值）
    public float SampleOffset(float localZ, float angle)
    {
        shape.GetBilinear(localZ, angle, out int i0, out int i1, out int j0, out int j1, out float fz, out float fa);

        return SampleOffset(i0, i1, j0, j1, fz, fa);
    }

    private float SampleOffset(int i0, int i1, int j0, int j1, float fz, float fa)
    {
        return
            offset[GetFieldIndex(i0, j0)] * (1f - fz) * (1f - fa)
            + offset[GetFieldIndex(i0, j1)] * (1f - fz) * fa
            + offset[GetFieldIndex(i1, j0)] * fz * (1f - fa)
            + offset[GetFieldIndex(i1, j1)] * fz * fa;
    }

    // 局部 (z, θ) 处的实际表面半径 = 基础半径 + 软组织偏移
    public float SampleSurfaceRadius(float localZ, float angle)
    {
        return shape.SampleBaseRadius(localZ, angle) + SampleOffset(localZ, angle);
    }

    public override bool ResolveContact(ref Vector3 worldPosition, float margin)
    {
        Vector3 localPosition = transform.InverseTransformPoint(worldPosition);

        if (localPosition.z < 0f || localPosition.z > shape.Length)
        {
            return false;
        }

        float currentRadius = Mathf.Sqrt(localPosition.x * localPosition.x + localPosition.y * localPosition.y);
        float angle = currentRadius < 0.00000001f ? 0f : Mathf.Atan2(localPosition.y, localPosition.x);

        // 每帧调用上万次：一次双线性查找，基础半径、软组织偏移共用
        shape.GetBilinear(localPosition.z, angle, out int i0, out int i1, out int j0, out int j1, out float fz, out float fa);

        float collisionRadius = shape.SampleBaseRadius(i0, i1, j0, j1, fz, fa) + SampleOffset(i0, i1, j0, j1, fz, fa) + margin;

        if (currentRadius >= collisionRadius)
        {
            return false;
        }

        // 沿径向推到表面外（推出距离即接触压力，由丝袜在解算结束后统一上报）
        if (currentRadius < 0.00000001f)
        {
            localPosition.x = collisionRadius;//先推出去
            localPosition.y = 0f;
        }
        else
        {
            float scale = collisionRadius / currentRadius;
            localPosition.x *= scale;
            localPosition.y *= scale;
        }

        worldPosition = transform.TransformPoint(localPosition);
        return true;
    }

    public override bool TryGetSurfaceFrame(Vector3 worldPosition, out LegSurfaceFrame frame)
    {
        shape.WorldToTube(worldPosition, out float localZ, out float angle, out float rho);

        if (rho < 0.001f)
        {
            frame = default;
            return false;
        }

        frame = shape.GetBaseSurfaceFrame(localZ, angle);
        return true;
    }

    // 射线与基础表面求交：先与外包圆柱求交得到进入段，再沿射线步进找"进入腿内"的位置并二分细化
    public override bool RaycastSurface(Ray worldRay, out Vector3 hitPoint)
    {
        hitPoint = default;

        Vector3 localOrigin = transform.InverseTransformPoint(worldRay.origin);
        Vector3 localDirection = transform.InverseTransformDirection(worldRay.direction).normalized;

        float boundRadius = shape.MaxBaseRadius;

        float a = localDirection.x * localDirection.x + localDirection.y * localDirection.y;
        float b = 2f * (localOrigin.x * localDirection.x + localOrigin.y * localDirection.y);
        float c = localOrigin.x * localOrigin.x + localOrigin.y * localOrigin.y - boundRadius * boundRadius;

        if (Mathf.Abs(a) < 0.000001f) return false;

        float discriminant = b * b - 4f * a * c;

        if (discriminant < 0f) return false;

        float sqrtDiscriminant = Mathf.Sqrt(discriminant);
        float tEnter = Mathf.Max(0f, (-b - sqrtDiscriminant) / (2f * a));
        float tExit = (-b + sqrtDiscriminant) / (2f * a);

        if (tExit < 0f) return false;

        float step = Mathf.Max(0.002f, 0.5f * shape.Length / (axialResolution - 1));
        float previous = tEnter;

        for (float t = tEnter; t <= tExit + step; t += step)
        {
            float tc = Mathf.Min(t, tExit);

            if (IsInsideBase(localOrigin + localDirection * tc))
            {
                // 圆柱时进入点就在表面上，直接命中；否则在 [previous, tc] 间二分细化
                if (tc > tEnter)
                {
                    float low = previous, high = tc;

                    for (int k = 0; k < 20; k++)
                    {
                        float mid = 0.5f * (low + high);
                        if (IsInsideBase(localOrigin + localDirection * mid)) high = mid; else low = mid;
                    }

                    tc = high;
                }

                hitPoint = transform.TransformPoint(localOrigin + localDirection * tc);
                return true;
            }

            previous = tc;

            if (tc >= tExit) break;
        }

        return false;
    }

    private bool IsInsideBase(Vector3 localPoint)
    {
        if (localPoint.z < 0f || localPoint.z > shape.Length)
        {
            return false;
        }

        float rho = Mathf.Sqrt(localPoint.x * localPoint.x + localPoint.y * localPoint.y);
        float angle = rho < 0.00000001f ? 0f : Mathf.Atan2(localPoint.y, localPoint.x);

        return rho <= shape.SampleBaseRadius(localPoint.z, angle) + 0.000001f;
    }

    // 丝袜上报本帧接触压力：压力按丝袜实际覆盖的面积铺到压力场上。
    // 只按粒子"点"采样时，粒子之间的格子没有压力，腿肉会在那里回弹成埂，
    // 布料被拖斜、拉长，或上层悬空的布盖在上面时，两粒子间的连线就会切进埂里。
    // 这里把三个顶点都贴腿的三角形覆盖的格子全部按顶点压力插值填满；
    // 悬空的布不参与，不会冲淡下层贴腿布料的压力。
    public override void SubmitContactPressure(Vector3[] worldPositions, float[] pressures, bool[] inContact, int[] triangles)
    {
        int count = worldPositions.Length;

        if (particleU == null || particleU.Length != count)
        {
            particleU = new float[count];
            particleV = new float[count];
        }

        Matrix4x4 worldToLocal = transform.worldToLocalMatrix;
        float rowScale = (axialResolution - 1) / Mathf.Max(shape.Length, 0.000001f);
        float columnScale = radialResolution / (2f * Mathf.PI);

        for (int i = 0; i < count; i++)
        {
            if (!inContact[i])
            {
                continue;
            }

            Vector3 local = worldToLocal.MultiplyPoint3x4(worldPositions[i]);
            float angle = Mathf.Atan2(local.y, local.x);

            particleU[i] = local.z * rowScale;
            particleV[i] = angle * columnScale;

            // 顶点本身按双线性采样，保证被压扁、窄于一格的三角形处也有压力
            if (local.z >= 0f && local.z <= shape.Length)
            {
                shape.GetBilinear(local.z, angle, out int i0, out int i1, out int j0, out int j1, out float fz, out float fa);
                AccumulatePressure(i0, i1, j0, j1, fz, fa, pressures[i]);
            }
        }

        for (int t = 0; t + 2 < triangles.Length; t += 3)
        {
            int a = triangles[t];
            int b = triangles[t + 1];
            int c = triangles[t + 2];

            if (inContact[a] && inContact[b] && inContact[c])
            {
                RasterizeTriangle(a, b, c, pressures[a], pressures[b], pressures[c]);
            }
        }
    }

    // 在 (行 u, 列 v) 网格上填充三角形覆盖的格点，压力按重心坐标插值
    private void RasterizeTriangle(int a, int b, int c, float pressureA, float pressureB, float pressureC)
    {
        float ua = particleU[a], va = particleV[a];
        float ub = particleU[b], vb = particleV[b];
        float uc = particleU[c], vc = particleV[c];

        // 绕腿方向首尾相接：以 A 为基准展开，三角形跨过 θ = 0 时不会被当成绕腿一整圈
        float half = radialResolution * 0.5f;
        vb = va + Mathf.Repeat(vb - va + half, radialResolution) - half;
        vc = va + Mathf.Repeat(vc - va + half, radialResolution) - half;

        // 两倍有向面积，退化三角形交给顶点采样
        float area = (ub - ua) * (vc - va) - (uc - ua) * (vb - va);

        if (Mathf.Abs(area) < 0.000001f)
        {
            return;
        }

        int iMin = Mathf.Max(0, Mathf.CeilToInt(Mathf.Min(ua, Mathf.Min(ub, uc))));
        int iMax = Mathf.Min(axialResolution - 1, Mathf.FloorToInt(Mathf.Max(ua, Mathf.Max(ub, uc))));
        int jMin = Mathf.CeilToInt(Mathf.Min(va, Mathf.Min(vb, vc)));
        int jMax = Mathf.FloorToInt(Mathf.Max(va, Mathf.Max(vb, vc)));

        // 异常拉扯出的超大三角形不铺（正常布料一个三角形只覆盖几个格点）
        if (iMax - iMin > axialResolution / 4 || jMax - jMin > radialResolution / 4)
        {
            return;
        }

        const float Epsilon = -0.0001f;
        float inverseArea = 1f / area;

        for (int i = iMin; i <= iMax; i++)
        {
            for (int j = jMin; j <= jMax; j++)
            {
                float weightA = ((ub - i) * (vc - j) - (uc - i) * (vb - j)) * inverseArea;
                float weightB = ((uc - i) * (va - j) - (ua - i) * (vc - j)) * inverseArea;
                float weightC = 1f - weightA - weightB;

                if (weightA < Epsilon || weightB < Epsilon || weightC < Epsilon)
                {
                    continue;
                }

                int column = ((j % radialResolution) + radialResolution) % radialResolution;

                AddSample(GetFieldIndex(i, column), weightA * pressureA + weightB * pressureB + weightC * pressureC, 1f);
            }
        }
    }

    private void AccumulatePressure(int i0, int i1, int j0, int j1, float fz, float fa, float pushDistance)
    {
        AddSample(GetFieldIndex(i0, j0), pushDistance, (1f - fz) * (1f - fa));
        AddSample(GetFieldIndex(i0, j1), pushDistance, (1f - fz) * fa);
        AddSample(GetFieldIndex(i1, j0), pushDistance, fz * (1f - fa));
        AddSample(GetFieldIndex(i1, j1), pushDistance, fz * fa);
    }

    private void AddSample(int index, float value, float weight)
    {
        pressureSum[index] += value * weight;
        pressureWeight[index] += weight;
    }

    // 所有 Update（丝袜模拟）之后，用本帧压力更新凹陷与鼓起
    private void LateUpdate()
    {
        float dt = Time.deltaTime;

        for (int i = 0; i < pressure.Length; i++)
        {
            pressure[i] = pressureWeight[i] > MinPressureWeight ? pressureSum[i] / pressureWeight[i] : 0f;
        }

        if (dt > 0f)
        {
            float blend = responseTime <= 0f ? 1f : 1f - Mathf.Exp(-dt / responseTime);

            for (int i = 0; i < depth.Length; i++)
            {
                float targetDepth = Mathf.Clamp(compliance * pressure[i], 0f, maxIndentation);

                depth[i] += (targetDepth - depth[i]) * blend;
            }
        }

        System.Array.Clear(pressureSum, 0, pressureSum.Length);
        System.Array.Clear(pressureWeight, 0, pressureWeight.Length);

        System.Array.Copy(depth, offset, depth.Length);
        Blur(offset, blurPasses);

        System.Array.Copy(offset, wideDepth, offset.Length);
        Blur(wideDepth, bulgeSpread);

        currentMaxIndentation = 0f;
        currentMaxBulge = 0f;

        for (int i = 0; i < offset.Length; i++)
        {
            float indentation = offset[i];
            float bulge = Mathf.Max(0f, wideDepth[i] - indentation) * bulgeStrength;

            offset[i] = bulge - indentation;

            currentMaxIndentation = Mathf.Max(currentMaxIndentation, indentation);
            currentMaxBulge = Mathf.Max(currentMaxBulge, bulge);
        }
    }

    // 可分离的 3 点盒式模糊：θ 方向首尾相接，z 方向边界截断
    private void Blur(float[] field, int passes)
    {
        for (int pass = 0; pass < passes; pass++)
        {
            for (int i = 0; i < axialResolution; i++)
            {
                for (int j = 0; j < radialResolution; j++)
                {
                    int prev = (j - 1 + radialResolution) % radialResolution;
                    int next = (j + 1) % radialResolution;

                    blurTemp[GetFieldIndex(i, j)] =
                        (field[GetFieldIndex(i, prev)]
                        + field[GetFieldIndex(i, j)]
                        + field[GetFieldIndex(i, next)]) / 3f;
                }
            }

            for (int i = 0; i < axialResolution; i++)
            {
                int prev = Mathf.Max(i - 1, 0);
                int next = Mathf.Min(i + 1, axialResolution - 1);

                for (int j = 0; j < radialResolution; j++)
                {
                    field[GetFieldIndex(i, j)] =
                        (blurTemp[GetFieldIndex(prev, j)]
                        + blurTemp[GetFieldIndex(i, j)]
                        + blurTemp[GetFieldIndex(next, j)]) / 3f;
                }
            }
        }
    }
}
