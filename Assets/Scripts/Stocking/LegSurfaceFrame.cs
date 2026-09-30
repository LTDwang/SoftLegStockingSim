using UnityEngine;

public struct LegSurfaceFrame
{
    // 腿表面上的参考点
    public Vector3 surfacePoint;

    // 表面法线：从腿内部指向外部
    public Vector3 normal;

    // 沿腿长度方向
    public Vector3 axialTangent;

    // 绕腿一圈的方向
    public Vector3 circumferentialTangent;
}