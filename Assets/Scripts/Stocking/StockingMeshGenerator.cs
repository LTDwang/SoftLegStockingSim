//丝袜网格生成：30 圈 × 60 点的管状网格，同时给出"自然形状"（约束原长的来源）
//指定腿形时沿腿生成，每圈垂直于腿中心线，截面按紧度等比缩小；否则生成固定半径的直圆筒

using UnityEngine;
using UnityEngine.Rendering;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class StockingMeshGenerator : MonoBehaviour
{
    [Header("Mesh Settings")]

    [SerializeField]
    [Tooltip("每圈的点数")]
    private int pointsInOneCircle = 32;

    [SerializeField]
    [Tooltip("圈数")]
    private int circleCount = 30;

    [SerializeField]
    [Tooltip("不受力时的自然半径（未指定腿形时使用）")]
    private float restRadius = 0.8f;

    [SerializeField]
    [Tooltip("初始生成半径（未指定腿形时使用）")]
    private float initialRadius = 1.05f;

    [SerializeField]
    [Tooltip("丝袜总长度")]
    private float stockingLength = 3f;

    [Header("Leg Fit")]

    [SerializeField]
    [Tooltip("沿哪条腿形生成；为空时生成直圆筒")]
    private TubeLegShape legShape;

    [SerializeField]
    [Tooltip("第 0 圈在腿形局部 z 上的位置")]
    private float startOnLeg = 0.5f;

    [SerializeField]
    [Range(0.3f, 1.2f)]
    [Tooltip("紧度：丝袜自然周长 / 腿周长，腿有粗细变化时各处勒紧程度一致")]
    private float tightness = 0.8f / 0.95f;

    [SerializeField]
    [Tooltip("初始生成时离开腿面的距离")]
    private float spawnGap = 0.1f;

    [Header("Material")]

    [SerializeField]
    private Material initialMaterial;

    [Header("Debug")]

    [SerializeField]
    private bool showGizmos = true;

    [SerializeField]
    private bool showRestShape = true;

    [SerializeField]
    private float gizmoPointSize = 0.025f;

    private Mesh mesh;

    // 当前顶点位置，由模拟每帧写回
    private Vector3[] vertices;

    // 自然形状，生成后不再改变
    private Vector3[] restVertices;

    private int[] triangles;

    private Vector2[] uvs;

    public Mesh Mesh => mesh;

    public Vector3[] Vertices => vertices;

    public Vector3[] RestVertices => restVertices;

    public int PointsInOneCircle => pointsInOneCircle;

    public int CircleCount => circleCount;

    public float RestRadius => restRadius;

    public float StockingLength => stockingLength;

    private void Awake()
    {
        GenerateMesh();
        SetupMaterial();
    }

    // =========================
    // Mesh Generation
    // =========================

    private void GenerateMesh()
    {
        mesh = new Mesh();
        mesh.name = "Stocking Simulation Mesh";

        int vertexCount = pointsInOneCircle * circleCount;

        // 默认 16 位索引，顶点超过 65535 时改用 32 位
        if (vertexCount > 65535)
        {
            mesh.indexFormat = IndexFormat.UInt32;
        }

        vertices = new Vector3[vertexCount];
        restVertices = new Vector3[vertexCount];

        GenerateVertices();
        GenerateTriangles();
        GenerateUVs();

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv = uvs;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        GetComponent<MeshFilter>().mesh = mesh;
    }

    // 直圆筒：XY 为截面，Z 为长度方向，第 0 圈在 z = 0
    private void GenerateVertices()
    {
        if (legShape != null)
        {
            GenerateVerticesOnLeg();
            return;
        }

        // 腿比丝袜粗时，初始位置直接生成在腿外
        float spawnRadius = Mathf.Max(restRadius, initialRadius);

        for (int circleIndex = 0; circleIndex < circleCount; circleIndex++)
        {
            float z = CalculateCircleZ(circleIndex);

            for (int pointIndex = 0; pointIndex < pointsInOneCircle; pointIndex++)
            {
                float angle = 2f * Mathf.PI * pointIndex / pointsInOneCircle;
                float cos = Mathf.Cos(angle);
                float sin = Mathf.Sin(angle);

                int vertexIndex = GetVertexIndex(circleIndex, pointIndex);

                restVertices[vertexIndex] = new Vector3(restRadius * cos, restRadius * sin, z);
                vertices[vertexIndex] = new Vector3(spawnRadius * cos, spawnRadius * sin, z);
            }
        }
    }

    // 沿腿形生成：自然形状 = 腿截面按紧度等比缩小，
    // 初始位置在腿面外 spawnGap 处（且不小于自然形状）
    private void GenerateVerticesOnLeg()
    {
        legShape.EnsureBuilt();

        for (int circleIndex = 0; circleIndex < circleCount; circleIndex++)
        {
            float legZ = startOnLeg + CalculateCircleZ(circleIndex);

            for (int pointIndex = 0; pointIndex < pointsInOneCircle; pointIndex++)
            {
                float angle = 2f * Mathf.PI * pointIndex / pointsInOneCircle;
                float legRadius = legShape.SampleBaseRadius(legZ, angle);

                float rest = tightness * legRadius;
                float spawn = Mathf.Max(rest, legRadius + spawnGap);

                int vertexIndex = GetVertexIndex(circleIndex, pointIndex);

                // 腿形局部 → 世界 → 丝袜局部
                restVertices[vertexIndex] = transform.InverseTransformPoint(legShape.TubeToWorld(legZ, angle, rest));
                vertices[vertexIndex] = transform.InverseTransformPoint(legShape.TubeToWorld(legZ, angle, spawn));
            }
        }
    }

    private float CalculateCircleZ(int circleIndex)
    {
        if (circleCount <= 1)
        {
            return 0f;
        }

        return stockingLength * circleIndex / (circleCount - 1f);
    }

    // 相邻两圈之间每个格子拆成两个三角形：
    //
    //   A ---- B      A = (c, p)      B = (c, p+1)
    //   |    / |      D = (c+1, p)    C = (c+1, p+1)
    //   |  /   |
    //   D ---- C      (A, B, C) 和 (A, C, D)，此绕序使法线朝外
    private void GenerateTriangles()
    {
        int quadCount = (circleCount - 1) * pointsInOneCircle;
        triangles = new int[quadCount * 6];

        int t = 0;

        for (int circleIndex = 0; circleIndex < circleCount - 1; circleIndex++)
        {
            for (int pointIndex = 0; pointIndex < pointsInOneCircle; pointIndex++)
            {
                int nextPoint = (pointIndex + 1) % pointsInOneCircle;

                int firstCurrent = GetVertexIndex(circleIndex, pointIndex);
                int firstNext = GetVertexIndex(circleIndex, nextPoint);
                int secondCurrent = GetVertexIndex(circleIndex + 1, pointIndex);
                int secondNext = GetVertexIndex(circleIndex + 1, nextPoint);

                triangles[t++] = firstCurrent;
                triangles[t++] = firstNext;
                triangles[t++] = secondNext;

                triangles[t++] = firstCurrent;
                triangles[t++] = secondNext;
                triangles[t++] = secondCurrent;
            }
        }
    }

    // u 绕圈、v 沿长度
    private void GenerateUVs()
    {
        uvs = new Vector2[vertices.Length];

        for (int circleIndex = 0; circleIndex < circleCount; circleIndex++)
        {
            float v = circleCount <= 1 ? 0f : (float)circleIndex / (circleCount - 1);

            for (int pointIndex = 0; pointIndex < pointsInOneCircle; pointIndex++)
            {
                float u = (float)pointIndex / pointsInOneCircle;

                uvs[GetVertexIndex(circleIndex, pointIndex)] = new Vector2(u, v);
            }
        }
    }

    public int GetVertexIndex(int circleIndex, int pointIndex)
    {
        return circleIndex * pointsInOneCircle + pointIndex;
    }

    // StockingSimulation 每帧解算完成后把粒子位置写回网格
    public void UpdateMeshVertices(Vector3[] newVertices)
    {
        if (newVertices == null)
        {
            return;
        }

        if (newVertices.Length != vertices.Length)
        {
            Debug.LogError("StockingMeshGenerator: 新的顶点数量和 Mesh 顶点数量不一致。");
            return;
        }

        vertices = newVertices;

        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    // =========================
    // Material
    // =========================

    private void SetupMaterial()
    {
        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();

        if (initialMaterial != null)
        {
            meshRenderer.material = initialMaterial;
            return;
        }

        Shader shader = Shader.Find("Universal Render Pipeline/Lit");

        if (shader == null)
        {
            shader = Shader.Find("Standard");
        }

        if (shader == null)
        {
            Debug.LogWarning("StockingMeshGenerator: 没有找到可用 Shader。");
            return;
        }

        Material material = new Material(shader);
        material.name = "Default Stocking Material";
        material.color = new Color(0.15f, 0.15f, 0.15f, 1f);

        meshRenderer.material = material;
    }

    // =========================
    // Gizmos
    // =========================

    // 黄点：当前粒子；绿线：同一圈；青线：相邻圈；红圈：自然形状
    private void OnDrawGizmos()
    {
        if (!showGizmos || vertices == null || vertices.Length == 0)
        {
            return;
        }

        Gizmos.color = Color.yellow;

        for (int i = 0; i < vertices.Length; i++)
        {
            Gizmos.DrawSphere(transform.TransformPoint(vertices[i]), gizmoPointSize);
        }

        Gizmos.color = Color.green;

        for (int circleIndex = 0; circleIndex < circleCount; circleIndex++)
        {
            for (int pointIndex = 0; pointIndex < pointsInOneCircle; pointIndex++)
            {
                int indexA = GetVertexIndex(circleIndex, pointIndex);
                int indexB = GetVertexIndex(circleIndex, (pointIndex + 1) % pointsInOneCircle);

                Gizmos.DrawLine(transform.TransformPoint(vertices[indexA]), transform.TransformPoint(vertices[indexB]));
            }
        }

        Gizmos.color = Color.cyan;

        for (int circleIndex = 0; circleIndex < circleCount - 1; circleIndex++)
        {
            for (int pointIndex = 0; pointIndex < pointsInOneCircle; pointIndex++)
            {
                int indexA = GetVertexIndex(circleIndex, pointIndex);
                int indexB = GetVertexIndex(circleIndex + 1, pointIndex);

                Gizmos.DrawLine(transform.TransformPoint(vertices[indexA]), transform.TransformPoint(vertices[indexB]));
            }
        }

        if (showRestShape && restVertices != null)
        {
            Gizmos.color = Color.red;

            for (int i = 0; i < restVertices.Length; i++)
            {
                Gizmos.DrawWireSphere(transform.TransformPoint(restVertices[i]), gizmoPointSize * 0.7f);
            }
        }
    }
}
