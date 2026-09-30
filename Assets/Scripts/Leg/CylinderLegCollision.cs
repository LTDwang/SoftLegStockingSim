//目前实现的第一版试验用腿部碰撞解算
//暂时以圆柱体代替大腿

using UnityEngine;

public class CylinderLegCollision : LegCollisionModel
{
    public float radius = 1f;
    public float length = 3f;

    public override bool RaycastSurface(Ray worldRay, out Vector3 hitPoint)
    {
        hitPoint = default;

        Vector3 localOrigin = transform.InverseTransformPoint(worldRay.origin);
        Vector3 localDirection = transform.InverseTransformDirection(worldRay.direction).normalized;

        float a = localDirection.x * localDirection.x + localDirection.y * localDirection.y;
        float b = 2f * (localOrigin.x * localDirection.x + localOrigin.y * localDirection.y);
        float c = localOrigin.x * localOrigin.x + localOrigin.y * localOrigin.y - radius * radius;

        if (Mathf.Abs(a) < 0.000001f) return false;

        float discriminant = b * b - 4f * a * c;

        if (discriminant < 0f) return false;

        float sqrtDiscriminant = Mathf.Sqrt(discriminant);

        float t1 = (-b - sqrtDiscriminant) / (2f * a);
        float t2 = (-b + sqrtDiscriminant) / (2f * a);

        float nearestT = float.PositiveInfinity;

        if (t1 >= 0f)
        {
            Vector3 localHit = localOrigin + localDirection * t1;

            if (localHit.z >= 0f && localHit.z <= length)
            {
                nearestT = t1;
            }
        }

        if (t2 >= 0f)
        {
            Vector3 localHit = localOrigin + localDirection * t2;

            if (localHit.z >= 0f && localHit.z <= length && t2 < nearestT)
            {
                nearestT = t2;
            }
        }

        if (float.IsPositiveInfinity(nearestT)) return false;

        Vector3 localHitPoint = localOrigin + localDirection * nearestT;

        hitPoint = transform.TransformPoint(localHitPoint);

        return true;
    }

    public override bool ResolveContact(ref Vector3 worldPosition, float margin)
    {
        Vector3 localPosition = transform.InverseTransformPoint(worldPosition);
        if (localPosition.z < 0f || localPosition.z > length)
        {
            return false;
        }
        float collisionRadius = radius + margin;
        float radiusSquared = localPosition.x * localPosition.x + localPosition.y * localPosition.y;
        float collisionRadiusSquared = collisionRadius * collisionRadius;

        if (radiusSquared >= collisionRadiusSquared)
        {
            return false;
        }
        float currentRadius = Mathf.Sqrt(radiusSquared);
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
        Vector3 localPosition =
            transform.InverseTransformPoint(
                worldPosition
            );

        Vector3 radialLocal =
            new Vector3(
                localPosition.x,
                localPosition.y,
                0f
            );

        if (radialLocal.sqrMagnitude < 0.000001f)
        {
            frame = default;
            return false;
        }

        radialLocal.Normalize();


        // 圆柱表面的外法线
        Vector3 normal =
            transform.TransformDirection(
                radialLocal
            ).normalized;


        // 当前圆柱沿自身局部 Z 方向延伸
        Vector3 axialTangent =
            transform.TransformDirection(
                Vector3.forward
            ).normalized;


        // 绕腿方向
        Vector3 circumferentialTangent =
            Vector3.Cross(
                axialTangent,
                normal
            ).normalized;


        // 算出真正位于圆柱表面的点
        Vector3 surfaceLocalPosition =
            new Vector3(
                radialLocal.x * radius,
                radialLocal.y * radius,
                localPosition.z
            );

        Vector3 surfaceWorldPosition =
            transform.TransformPoint(
                surfaceLocalPosition
            );


        frame =
            new LegSurfaceFrame
            {
                surfacePoint =
                    surfaceWorldPosition,

                normal =
                    normal,

                axialTangent =
                    axialTangent,

                circumferentialTangent =
                    circumferentialTangent
            };

        return true;
    }
}