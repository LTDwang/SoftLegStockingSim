using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StockingSimulation : MonoBehaviour
{
    [SerializeField]
    private StockingMeshGenerator meshGenerator;

    [SerializeField]
    [Range(0f,1f)]
    private float circumferentialStiffness = 0.8f;

    [SerializeField]
    [Range(0f, 1f)]
    private float longitudinalStiffness = 0.8f;

    [SerializeField]
    [Range(0f, 1f)]
    private float shearStiffness = 0.5f;

    [SerializeField]
    [Range(0f, 1f)]
    [Tooltip("抗弯（防翻穿）约束强度：纵向隔一圈、同圈隔一点的约束，只在折得过狠时起作用")]
    private float bendStiffness = 0.4f;

    [SerializeField]
    [Range(0f, 1f)]
    [Tooltip("抗弯约束起效比例：隔一圈/隔一点两端的距离被压到原长的该比例以下时才推回，且只推回到该比例；比例以上的普通褶皱不回弹")]
    private float minFoldRatio = 0.5f;

    [Header("Leg Friction")]

    [SerializeField]
    [Tooltip("静摩擦阈值：贴腿粒子一帧内沿腿面的滑动小于该距离时被粘住，回弹力弱的褶皱因此不会完全恢复")]
    private float staticFrictionDistance = 0.002f;

    [SerializeField]
    [Range(0f, 1f)]
    [Tooltip("动摩擦系数：滑动超过静摩擦阈值时，沿腿面的滑动被削减的比例")]
    private float kineticFriction = 0.3f;

    [SerializeField]
    private float collisionMargin = 0.01f;

    [SerializeField]
    [Range(1,20)]
    [Tooltip("每一帧计算的次数")]
    private int solverIterations = 5;

    [SerializeField]
    private LegCollisionModel legCollision;

    [SerializeField]
    [Range (0f, 1f)]
    private float dragStiffness = 0.3f;

    [SerializeField]
    private float maxDragStep = 0.01f;

    [SerializeField]
    private float maxDragTargetDistance = 0.5f;

    private int draggedParticleIndex = -1;

    private Vector3 dragTargetWorldPosition;

    private Vector3[] meshVertexBuffer;//要传回mesh的顶点组

    private StockingParticle[] particles;

    private List<StockingConstraint> constraints;

    // 本帧是否与腿发生过接触（用于摩擦、压力上报）
    private bool[] legContact;

    // 本帧各次迭代被腿推出的距离之和，除以迭代次数即接触压力
    private float[] contactPush;
    private float[] contactPressure;

    // 粒子世界坐标的临时缓冲（边中点防穿透、压力上报共用）
    private Vector3[] worldPositionBuffer;

    // 丝袜网格三角形，压力按三角形覆盖的面积上报给腿
    private int[] meshTriangles;

    // 丝袜网格的边（端点索引成对存放），用于边中点防穿透
    private int[] meshEdges;

    private bool[] edgePushed;

    // 边中点与腿面保持的距离（相对 collisionMargin 的比例）。
    // 腿是弯的，同圈相邻两点间的弦本来就比两端低一点（约 0.0013），
    // 用满额间距会让静置时每条边都被抬起，丝袜整体浮起、勒痕变浅
    private const float EdgeMarginRatio = 0.5f;

    // Start is called before the first frame update
    void Start()
    {
        meshGenerator = GetComponent<StockingMeshGenerator>();
        InitializeParticles();
        InitializeConstraints();

        meshVertexBuffer = new Vector3[particles.Length];
        legContact = new bool[particles.Length];
        contactPush = new float[particles.Length];
        contactPressure = new float[particles.Length];
        worldPositionBuffer = new Vector3[particles.Length];
        edgePushed = new bool[particles.Length];
        meshTriangles = meshGenerator.Mesh.triangles;
        meshEdges = BuildMeshEdges(meshTriangles);
    }

    // 从三角形索引中取出不重复的边（同圈、相邻两圈、三角形对角线）
    private static int[] BuildMeshEdges(int[] triangles)
    {
        HashSet<long> seen = new HashSet<long>();
        List<int> edges = new List<int>();

        for (int t = 0; t + 2 < triangles.Length; t += 3)
        {
            for (int k = 0; k < 3; k++)
            {
                int a = triangles[t + k];
                int b = triangles[t + (k + 1) % 3];
                long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;

                if (seen.Add(key))
                {
                    edges.Add(a);
                    edges.Add(b);
                }
            }
        }

        return edges.ToArray();
    }

    // Update is called once per frame
    void Update()
    {
        SolveSimulation();
        UpdateMesh();
    }

    public int ParticleCount
    {
        get
        {
            return particles == null ? 0 : particles.Length;
        }
    }

    public Vector3 GetParticleWorldPosition(int index)
    {
        return meshGenerator.transform.TransformPoint(particles[index].position);
    }


    public void BeginDrag(int particleIndex, Vector3 targetWorldPosition)
    {
        draggedParticleIndex = particleIndex;
        dragTargetWorldPosition = targetWorldPosition;
    }

    public void UpdateDragTarget(Vector3 targetWorldPosition)
    {
        dragTargetWorldPosition = targetWorldPosition;
    }

    public void EndDrag()
    {
        draggedParticleIndex = -1;
    }

    private void SolveMouseDrag()
    {
        if (draggedParticleIndex < 0) return;
        StockingParticle particle = particles[draggedParticleIndex];
        Vector3 targetLocalPosition = meshGenerator.transform.InverseTransformPoint(dragTargetWorldPosition);
        Vector3 offset = targetLocalPosition - particle.position;
        float distance = offset.magnitude;
        if (distance < 0.000001f)
        {
            return;
        }
        if (distance > maxDragTargetDistance)
        {
            offset = offset.normalized * maxDragTargetDistance;
            distance = maxDragTargetDistance;
        }
        float moveDistance = Mathf.Min(distance * dragStiffness, maxDragStep);
        particle.position += offset.normalized * moveDistance;
        particles[draggedParticleIndex] = particle;
    }

    private void UpdateMesh()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            meshVertexBuffer[i] = particles[i].position;
        }

        meshGenerator.UpdateMeshVertices( meshVertexBuffer );
    }

    private void SolveSimulation()
    {
        if (constraints == null)
            return;

        // 记录本帧起始位置，摩擦据此计算本帧沿腿面的滑动
        for (int i = 0; i < particles.Length; i++)
        {
            particles[i].previousPosition = particles[i].position;
            legContact[i] = false;
            contactPush[i] = 0f;
        }

        for (int iteration = 0; iteration < solverIterations; iteration++)
        {
            // 正反向交替遍历约束：固定顺序会让修正总是偏向同一端，
            // 导致一端的翻折能恢复、另一端却越卷越多
            bool reverse = (iteration & 1) == 1;
            int count = constraints.Count;
            for (int i = 0; i < count; i++)
            {
                SolveConstraint(constraints[reverse ? count - 1 - i : i]);
            }
            SolveMouseDrag();
            SolveLegCollision();
        }

        SolveEdgeCollision();
        ApplyLegFriction();
        SubmitContactPressure();
    }

    // 边中点防穿透：粒子只在顶点处与腿碰撞，腿面在两粒子之间比两端高时
    // （勒肉鼓起、凹陷还没跟上快速拖动的布料、布料被拉长斜跨过弯曲的腿面），
    // 两点间的连线会切进腿里。这里检查每条边的中点，陷进腿面时把两端沿推出方向一起抬起
    private void SolveEdgeCollision()
    {
        if (legCollision == null)
        {
            return;
        }

        Transform meshTransform = meshGenerator.transform;
        Matrix4x4 localToWorld = meshTransform.localToWorldMatrix;
        float edgeMargin = collisionMargin * EdgeMarginRatio;

        // 先统一换到世界坐标，每条边只需一次碰撞查询
        for (int i = 0; i < particles.Length; i++)
        {
            worldPositionBuffer[i] = localToWorld.MultiplyPoint3x4(particles[i].position);
            edgePushed[i] = false;
        }

        for (int e = 0; e + 1 < meshEdges.Length; e += 2)
        {
            int a = meshEdges[e];
            int b = meshEdges[e + 1];

            Vector3 midpoint = (worldPositionBuffer[a] + worldPositionBuffer[b]) * 0.5f;
            Vector3 resolved = midpoint;

            if (!legCollision.ResolveContact(ref resolved, edgeMargin))
            {
                continue;
            }

            Vector3 push = resolved - midpoint;

            if (!particles[a].isFixed)
            {
                worldPositionBuffer[a] += push;
                edgePushed[a] = true;
            }

            if (!particles[b].isFixed)
            {
                worldPositionBuffer[b] += push;
                edgePushed[b] = true;
            }
        }

        Matrix4x4 worldToLocal = meshTransform.worldToLocalMatrix;

        for (int i = 0; i < particles.Length; i++)
        {
            if (edgePushed[i])
            {
                particles[i].position = worldToLocal.MultiplyPoint3x4(worldPositionBuffer[i]);
            }
        }
    }

    // 把本帧的接触压力交给腿（软组织据此计算凹陷）。
    // 压力 = 各次迭代被推出距离的平均值；本帧没贴过腿的粒子（例如被拉起、叠在上层的布）不上报
    private void SubmitContactPressure()
    {
        if (legCollision == null)
        {
            return;
        }

        Matrix4x4 localToWorld = meshGenerator.transform.localToWorldMatrix;
        float inverseIterations = 1f / solverIterations;

        for (int i = 0; i < particles.Length; i++)
        {
            worldPositionBuffer[i] = localToWorld.MultiplyPoint3x4(particles[i].position);
            contactPressure[i] = contactPush[i] * inverseIterations;
        }

        legCollision.SubmitContactPressure(worldPositionBuffer, contactPressure, legContact, meshTriangles);
    }

    // 腿面摩擦：本帧贴腿的粒子，沿腿面的滑动小于静摩擦阈值时撤销（粘住），
    // 否则按动摩擦系数削减。回弹力弱的褶皱会被留住，拉伸处拉力大仍能基本恢复
    private void ApplyLegFriction()
    {
        if (legCollision == null)
        {
            return;
        }

        Transform meshTransform = meshGenerator.transform;

        for (int i = 0; i < particles.Length; i++)
        {
            // 被拖拽的粒子不受摩擦
            if (!legContact[i] || i == draggedParticleIndex)
            {
                continue;
            }

            StockingParticle particle = particles[i];

            if (particle.isFixed)
            {
                continue;
            }

            Vector3 worldPosition = meshTransform.TransformPoint(particle.position);

            if (!legCollision.TryGetSurfaceFrame(worldPosition, out LegSurfaceFrame frame))
            {
                continue;
            }

            Vector3 normal = meshTransform.InverseTransformDirection(frame.normal).normalized;
            Vector3 displacement = particle.position - particle.previousPosition;
            Vector3 tangential = displacement - normal * Vector3.Dot(displacement, normal);

            if (tangential.magnitude < staticFrictionDistance)
            {
                particle.position -= tangential;
            }
            else
            {
                particle.position -= tangential * kineticFriction;
            }

            particles[i] = particle;
        }
    }

    private void SolveLegCollision()
    {
        if (legCollision == null)
        {
            return;
        }

        for (int i = 0; i < particles.Length; i++)
        {
            StockingParticle particle =
                particles[i];

            if (particle.isFixed)
            {
                continue;
            }

            Vector3 worldPosition =
                meshGenerator.transform.TransformPoint(
                    particle.position
                );

            Vector3 beforeContact = worldPosition;

            if (legCollision.ResolveContact(
                ref worldPosition,
                collisionMargin))
            {
                particle.position =
                    meshGenerator.transform.InverseTransformPoint(
                        worldPosition
                    );

                particles[i] = particle;
                legContact[i] = true;
                contactPush[i] += Vector3.Distance(beforeContact, worldPosition);
            }
        }
    }
    private void SolveConstraint(StockingConstraint constraint)
    {
        int indexA = constraint.particleA;
        int indexB = constraint.particleB;
        StockingParticle particleA = particles[indexA];
        StockingParticle particleB = particles[indexB];
        Vector3 delta = particleB.position - particleA.position;//A指向B
        float currentLength = delta.magnitude;
        if (currentLength < 0.000001f)
            return;
        float error = currentLength - constraint.originalLength;
        if (constraint.type == StockingConstraintType.LongitudinalBend
            || constraint.type == StockingConstraintType.CircumferentialBend)
        {
            // 抗弯约束只防"压平翻穿"：距离低于原长的 minFoldRatio 时才推回，
            // 且只推回到该比例，普通褶皱保持原样
            float foldLimit = constraint.originalLength * minFoldRatio;
            if (currentLength >= foldLimit)
            {
                return;
            }
            error = currentLength - foldLimit;
        }
        if (constraint.type == StockingConstraintType.Circumferential && error <= 0f)
        {
            return;
        }
        Vector3 direction = delta.normalized;
        Vector3 correction = direction * error * constraint.stiffness;
        if (particleA.isFixed && particleB.isFixed)
        {
            return;
        }
        else if (particleA.isFixed)
        {
            particleB.position -= correction;
        }
        else if (particleB.isFixed)
        {
            particleA.position += correction;
        }
        else
        {
            particleA.position += correction * 0.5f;
            particleB.position -= correction * 0.5f;
        }
        particles[indexA] = particleA;
        particles[indexB] = particleB;
    }

    private void InitializeParticles()
    {
        Vector3[] vertices = meshGenerator.Vertices;
        particles = new StockingParticle[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            particles[i] = new StockingParticle(vertices[i]);
        }
    }

    private void InitializeConstraints()
    {
        constraints = new List<StockingConstraint>();
        Vector3[] fittedVertices = ComputeFittedVertices();
        GenerateCircumferentialConstraints();
        GenerateLongitudinalConstraints();
        GenerateShearConstraints(fittedVertices);
        GenerateLongitudinalBendConstraints(fittedVertices);
        GenerateCircumferentialBendConstraints();
    }

    // 丝袜穿在腿上时的形状：自然形状的点沿腿表面法线移到腿表面外 collisionMargin 处。
    // 斜向与纵向抗弯约束以它为原长；若按自然半径计算，斜向约束会把丝袜纵向压短，
    // 带着纵向压力的布料没有抗弯能力时很容易被挤折。
    private Vector3[] ComputeFittedVertices()
    {
        Vector3[] restVertices = meshGenerator.RestVertices;
        Vector3[] fitted = new Vector3[restVertices.Length];
        Transform meshTransform = meshGenerator.transform;

        for (int i = 0; i < restVertices.Length; i++)
        {
            fitted[i] = restVertices[i];

            if (legCollision == null)
            {
                continue;
            }

            Vector3 worldPosition = meshTransform.TransformPoint(restVertices[i]);

            if (!legCollision.TryGetSurfaceFrame(worldPosition, out LegSurfaceFrame frame))
            {
                continue;
            }

            // 自然形状本来就比腿松的地方保持原样
            float heightAboveSurface = Vector3.Dot(worldPosition - frame.surfacePoint, frame.normal);

            if (heightAboveSurface < collisionMargin)
            {
                fitted[i] = meshTransform.InverseTransformPoint(frame.surfacePoint + frame.normal * collisionMargin);
            }
        }

        return fitted;
    }

    private void GenerateShearConstraints(Vector3[] fittedVertices)//对角线
    {
        int circleCount = meshGenerator.CircleCount;
        int pointsInCircle = meshGenerator.PointsInOneCircle;
        for (int circleIndex = 0; circleIndex < circleCount - 1; circleIndex++)
        {
            for (int pointIndex = 0; pointIndex < pointsInCircle; pointIndex++)
            {
                int nextPoint = (pointIndex + 1) % pointsInCircle;
                int currentA = meshGenerator.GetVertexIndex(circleIndex, pointIndex);
                int currentB = meshGenerator.GetVertexIndex(circleIndex, nextPoint);
                int nextA = meshGenerator.GetVertexIndex(circleIndex + 1, pointIndex);
                int nextB = meshGenerator.GetVertexIndex(circleIndex + 1, nextPoint);
                AddConstraint(currentA, nextB, StockingConstraintType.Shear, shearStiffness, fittedVertices);
                AddConstraint(currentB, nextA, StockingConstraintType.Shear, shearStiffness, fittedVertices);

            }
        }
    }

    private void GenerateLongitudinalBendConstraints(Vector3[] fittedVertices)//纵向隔一圈
    {
        // 某一圈翻过相邻圈时，这条约束两端的距离会从两个圈距塌缩到接近 0，
        // 从而产生强恢复力；普通距离约束在翻折状态下同样满足，无法把它拉回来
        int circleCount = meshGenerator.CircleCount;
        int pointsInCircle = meshGenerator.PointsInOneCircle;
        for (int circleIndex = 0; circleIndex < circleCount - 2; circleIndex++)
        {
            for (int pointIndex = 0; pointIndex < pointsInCircle; pointIndex++)
            {
                int indexA = meshGenerator.GetVertexIndex(circleIndex, pointIndex);
                int indexB = meshGenerator.GetVertexIndex(circleIndex + 2, pointIndex);
                AddConstraint(indexA, indexB, StockingConstraintType.LongitudinalBend, bendStiffness, fittedVertices);
            }
        }
    }

    private void GenerateCircumferentialBendConstraints()//同圈隔一点
    {
        // 原长取自然形状，只抗压（被挤到自然间距的 minFoldRatio 以下才起作用），
        // 防止粒子交叉，同时不增加穿着时的环向收紧力
        int circleCount = meshGenerator.CircleCount;
        int pointsInCircle = meshGenerator.PointsInOneCircle;
        for (int circleIndex = 0; circleIndex < circleCount; circleIndex++)
        {
            for (int pointIndex = 0; pointIndex < pointsInCircle; pointIndex++)
            {
                int skipPoint = (pointIndex + 2) % pointsInCircle;
                int indexA = meshGenerator.GetVertexIndex(circleIndex, pointIndex);
                int indexB = meshGenerator.GetVertexIndex(circleIndex, skipPoint);
                AddConstraint(indexA, indexB, StockingConstraintType.CircumferentialBend, bendStiffness);
            }
        }
    }

    private void GenerateLongitudinalConstraints()//垂线
    {
        int circleCount = meshGenerator.CircleCount;
        int pointsInCircle = meshGenerator.PointsInOneCircle;
        for (int circleIndex = 0; circleIndex < circleCount - 1; circleIndex++)
        {
            for (int pointIndex = 0; pointIndex < pointsInCircle; pointIndex++)
            {
                int indexA = meshGenerator.GetVertexIndex(circleIndex, pointIndex);
                int indexB = meshGenerator.GetVertexIndex(circleIndex+1, pointIndex);
                AddConstraint(indexA, indexB, StockingConstraintType.Longitudinal, longitudinalStiffness);
            }
        }
    }

    private void GenerateCircumferentialConstraints()//同圈
    {
        int circleCount = meshGenerator.CircleCount;
        int pointsInCircle = meshGenerator.PointsInOneCircle;
        for (int circleIndex = 0; circleIndex < circleCount; circleIndex++)
        {
            for (int pointIndex = 0; pointIndex < pointsInCircle; pointIndex++)
            {
                int nextPoint = (pointIndex + 1) % pointsInCircle;
                int indexA = meshGenerator.GetVertexIndex(circleIndex, pointIndex);
                int indexB = meshGenerator.GetVertexIndex(circleIndex, nextPoint);
                AddConstraint(indexA, indexB, StockingConstraintType.Circumferential, circumferentialStiffness);
            }
        }
    }

    private void AddConstraint(int particleA, int particleB, StockingConstraintType type, float stiffness, Vector3[] referenceShape = null)
    {
        Vector3[] shape = referenceShape ?? meshGenerator.RestVertices;
        float restLength = Vector3.Distance(shape[particleA], shape[particleB]);
        StockingConstraint stockingConstraint = new StockingConstraint(type, particleA, particleB, restLength, stiffness);
        constraints.Add(stockingConstraint);
    }

}
