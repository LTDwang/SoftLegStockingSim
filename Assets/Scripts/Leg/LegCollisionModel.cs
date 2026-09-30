using UnityEngine;

/// <summary>
/// 腿碰撞模型的统一基类。
///
/// StockingSimulation 不需要知道腿具体是什么形状，
/// 只需要调用 ResolveContact()。
///
/// 以后可以有：
/// CylinderLegCollision
/// SDFLegCollision
/// SoftTissueLegCollision
/// 等不同实现。
/// </summary>
public abstract class LegCollisionModel : MonoBehaviour
{
    /// <summary>
    /// 处理丝袜粒子与腿之间的接触。
    ///
    /// worldPosition:
    ///     输入时是丝袜粒子的世界坐标。
    ///     如果发生碰撞，会被修改成修正后的世界坐标。
    ///
    /// margin:
    ///     丝袜和腿表面之间预留的一点距离，
    ///     防止数值误差导致反复穿透。
    ///
    /// 返回值：
    ///     true  = 发生了接触并修改了位置。
    ///     false = 没有发生接触。
    /// </summary>
    public abstract bool ResolveContact(
        ref Vector3 worldPosition,
        float margin
    );
    public abstract bool TryGetSurfaceFrame(
        Vector3 worldPosition,out LegSurfaceFrame frame
    );

    public abstract bool RaycastSurface(Ray worldRay, out Vector3 hitPoint);

    /// <summary>
    /// 丝袜每帧解算结束后上报本帧的接触压力，软组织腿据此计算凹陷；刚性腿忽略。
    ///
    /// worldPositions:  每个粒子的世界坐标。
    /// pressures:       每个粒子的接触压力（本帧各次迭代被推出距离的平均值）。
    /// inContact:       本帧是否贴过腿；没贴过的粒子不参与。
    /// triangles:       丝袜网格的三角形索引，三个顶点都贴腿的三角形按面积铺满压力。
    /// </summary>
    public virtual void SubmitContactPressure(
        Vector3[] worldPositions,
        float[] pressures,
        bool[] inContact,
        int[] triangles
    )
    {
    }
}