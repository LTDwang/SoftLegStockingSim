//软组织腿的外观网格（程序生成）：顶点与腿形网格一一对应，半径 = 基础半径 r0 + 软组织偏移
//在 SoftTissueLegCollision.LateUpdate 之前执行：显示的是丝袜本帧碰撞所用的那份偏移场，
//否则画面上是"新腿面 + 按旧腿面解算的丝袜"，拖拽时腿面回弹会顶穿丝袜
//以后导入腿模型时，由"模型变形"显示组件代替本组件

using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
[RequireComponent(typeof(SoftTissueLegCollision))]
[DefaultExecutionOrder(-100)]
public class SoftLegVisual : MonoBehaviour
{
    [SerializeField]
    private bool generateCaps = true;

    private SoftTissueLegCollision leg;

    private TubeLegShape shape;

    private Mesh mesh;

    private Vector3[] vertices;

    private float[] cosTable;
    private float[] sinTable;

    private int sideVertexCount;

    private void Start()
    {
        leg = GetComponent<SoftTissueLegCollision>();
        shape = GetComponent<TubeLegShape>();
        shape.EnsureBuilt();
        GenerateMesh();
    }

    private void GenerateMesh()
    {
        int rings = leg.AxialResolution;
        int points = leg.RadialResolution;

        cosTable = new float[points];
        sinTable = new float[points];

        for (int j = 0; j < points; j++)
        {
            float angle = 2f * Mathf.PI * j / points;
            cosTable[j] = Mathf.Cos(angle);
            sinTable[j] = Mathf.Sin(angle);
        }

        sideVertexCount = rings * points;

        // 封口：两端各一圈独立顶点 + 一个中心点，保持硬边
        int capVertexCount = generateCaps ? 2 * (points + 1) : 0;

        vertices = new Vector3[sideVertexCount + capVertexCount];

        int quadCount = (rings - 1) * points;
        int capTriangleCount = generateCaps ? 2 * points : 0;
        int[] triangles = new int[quadCount * 6 + capTriangleCount * 3];

        int t = 0;

        for (int i = 0; i < rings - 1; i++)
        {
            for (int j = 0; j < points; j++)
            {
                int next = (j + 1) % points;

                int firstCurrent = leg.GetFieldIndex(i, j);
                int firstNext = leg.GetFieldIndex(i, next);
                int secondCurrent = leg.GetFieldIndex(i + 1, j);
                int secondNext = leg.GetFieldIndex(i + 1, next);

                // 与 StockingMeshGenerator 相同的绕序，法线朝外
                triangles[t++] = firstCurrent;
                triangles[t++] = firstNext;
                triangles[t++] = secondNext;

                triangles[t++] = firstCurrent;
                triangles[t++] = secondNext;
                triangles[t++] = secondCurrent;
            }
        }

        if (generateCaps)
        {
            int bottomStart = sideVertexCount;
            int bottomCenter = bottomStart + points;
            int topStart = bottomCenter + 1;
            int topCenter = topStart + points;

            for (int j = 0; j < points; j++)
            {
                int next = (j + 1) % points;

                // 底面（z = 0）朝 -Z
                triangles[t++] = bottomCenter;
                triangles[t++] = bottomStart + next;
                triangles[t++] = bottomStart + j;

                // 顶面（z = length）朝 +Z
                triangles[t++] = topCenter;
                triangles[t++] = topStart + j;
                triangles[t++] = topStart + next;
            }
        }

        UpdateVertices();

        mesh = new Mesh();
        mesh.name = "Soft Leg Mesh";
        mesh.MarkDynamic();

        if (vertices.Length > 65535)
        {
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        GetComponent<MeshFilter>().mesh = mesh;
    }

    private void LateUpdate()
    {
        if (mesh == null)
        {
            return;
        }

        UpdateVertices();

        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    private void UpdateVertices()
    {
        int rings = leg.AxialResolution;
        int points = leg.RadialResolution;

        for (int i = 0; i < rings; i++)
        {
            float z = shape.Length * i / (rings - 1f);

            for (int j = 0; j < points; j++)
            {
                float r = shape.GetBaseRadius(i, j) + leg.GetRadialOffset(i, j);
                vertices[leg.GetFieldIndex(i, j)] = new Vector3(r * cosTable[j], r * sinTable[j], z);
            }
        }

        if (!generateCaps)
        {
            return;
        }

        int bottomStart = sideVertexCount;
        int bottomCenter = bottomStart + points;
        int topStart = bottomCenter + 1;
        int topCenter = topStart + points;

        for (int j = 0; j < points; j++)
        {
            vertices[bottomStart + j] = vertices[leg.GetFieldIndex(0, j)];
            vertices[topStart + j] = vertices[leg.GetFieldIndex(rings - 1, j)];
        }

        vertices[bottomCenter] = Vector3.zero;
        vertices[topCenter] = new Vector3(0f, 0f, shape.Length);
    }
}
