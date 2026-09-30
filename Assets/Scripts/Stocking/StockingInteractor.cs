using UnityEngine;
using UnityEngine.InputSystem;

public class StockingInteractor : MonoBehaviour
{
    [SerializeField]
    private StockingSimulation simulation;

    [SerializeField]
    private Camera targetCamera;

    [SerializeField]
    private LegCollisionModel legCollision;

    [SerializeField]
    private float selectionRadiusPixels = 25f;

    [SerializeField]
    [Range(0f, 1f)]
    [Tooltip("当射线与正常拖拽平面接近平行时，切换到摄像机友好的备用平面")]
    private float parallelThreshold = 0.2f;

    [SerializeField]
    [Range(-1f, 1f)]
    private float frontFacingThreshold = 0f;

    [SerializeField]
    [Tooltip("指针屏幕坐标输入，留空时使用项目级 Input Actions 中的 UI/Point")]
    private InputActionReference pointAction;

    [SerializeField]
    [Tooltip("拖拽按键输入，留空时使用项目级 Input Actions 中的 UI/Click")]
    private InputActionReference clickAction;

    private InputAction point;
    private InputAction click;

    private int selectedParticle = -1;

    private void Awake()
    {
        if (simulation == null)
        {
            simulation = GetComponent<StockingSimulation>();
        }

        if (targetCamera == null)
        {
            targetCamera = Camera.main;
        }

        point = ResolveAction(pointAction, "UI/Point");
        click = ResolveAction(clickAction, "UI/Click");
    }

    private void OnEnable()
    {
        point?.Enable();
        click?.Enable();
    }

    private static InputAction ResolveAction(InputActionReference reference, string fallbackPath)
    {
        if (reference != null && reference.action != null)
        {
            return reference.action;
        }

        InputAction action = InputSystem.actions != null ? InputSystem.actions.FindAction(fallbackPath) : null;

        if (action == null)
        {
            Debug.LogError($"StockingInteractor: 找不到输入动作 {fallbackPath}");
        }

        return action;
    }

    private Vector2 PointerPosition => point != null ? point.ReadValue<Vector2>() : Vector2.zero;

    private void Update()
    {
        if (point == null || click == null)
        {
            return;
        }

        if (click.WasPressedThisFrame() && selectedParticle < 0)
        {
            TryBeginDrag();
        }

        if (selectedParticle >= 0 && click.IsPressed())
        {
            UpdateDrag();
        }

        if (click.WasReleasedThisFrame())
        {
            EndDrag();
        }
    }

    private void TryBeginDrag()
    {
        if (simulation == null || targetCamera == null || legCollision == null)
        {
            return;
        }

        int particleIndex = FindNearestParticle();

        if (particleIndex < 0)
        {
            return;
        }

        Vector3 particleWorldPosition = simulation.GetParticleWorldPosition(particleIndex);

        if (!legCollision.TryGetSurfaceFrame(particleWorldPosition, out _))
        {
            return;
        }

        selectedParticle = particleIndex;

        simulation.BeginDrag(particleIndex, particleWorldPosition);
    }

    private void UpdateDrag()
    {
        if (selectedParticle < 0)
        {
            return;
        }

        Vector3 currentParticleWorldPosition = simulation.GetParticleWorldPosition(selectedParticle);

        if (!legCollision.TryGetSurfaceFrame(currentParticleWorldPosition, out LegSurfaceFrame frame))
        {
            return;
        }

        Vector3 axisDirection = frame.axialTangent.normalized;
        Vector3 radialDirection = frame.normal.normalized;

        Ray ray = targetCamera.ScreenPointToRay(PointerPosition);

        Vector3 planeNormal = Vector3.Cross(axisDirection, radialDirection).normalized;

        float rayPlaneAlignment = Mathf.Abs(Vector3.Dot(ray.direction, planeNormal));

        if (rayPlaneAlignment < parallelThreshold)
        {
            planeNormal = GetCameraFriendlyPlaneNormal(axisDirection);

            if (planeNormal.sqrMagnitude < 0.000001f)
            {
                return;
            }
        }

        Plane currentDragPlane = new Plane(planeNormal, currentParticleWorldPosition);

        if (!currentDragPlane.Raycast(ray, out float distance))
        {
            return;
        }

        Vector3 targetWorldPosition = ray.GetPoint(distance);

        if (legCollision.RaycastSurface(ray, out Vector3 legHitPoint))
        {
            float legHitDistance = Vector3.Distance(ray.origin, legHitPoint);

            if (legHitDistance < distance)
            {
                return;
            }
        }


        simulation.UpdateDragTarget(targetWorldPosition);
    }

    private Vector3 GetCameraFriendlyPlaneNormal(Vector3 axisDirection)
    {
        Vector3 planeNormal = Vector3.ProjectOnPlane(targetCamera.transform.forward, axisDirection);

        if (planeNormal.sqrMagnitude < 0.000001f)
        {
            planeNormal = Vector3.ProjectOnPlane(targetCamera.transform.up, axisDirection);
        }

        if (planeNormal.sqrMagnitude < 0.000001f)
        {
            planeNormal = Vector3.ProjectOnPlane(targetCamera.transform.right, axisDirection);
        }

        if (planeNormal.sqrMagnitude < 0.000001f)
        {
            return Vector3.zero;
        }

        return planeNormal.normalized;
    }

    private void EndDrag()
    {
        if (selectedParticle < 0)
        {
            return;
        }

        selectedParticle = -1;

        simulation.EndDrag();
    }

    private int FindNearestParticle()
    {
        Vector2 mousePosition = PointerPosition;

        int nearestParticle = -1;
        float nearestDistance = selectionRadiusPixels;

        for (int i = 0; i < simulation.ParticleCount; i++)
        {
            Vector3 worldPosition = simulation.GetParticleWorldPosition(i);
            Vector3 screenPosition = targetCamera.WorldToScreenPoint(worldPosition);

            if (screenPosition.z <= 0f)
            {
                continue;
            }

            if (!legCollision.TryGetSurfaceFrame(worldPosition, out LegSurfaceFrame frame))
            {
                continue;
            }

            Vector3 viewDirection = (targetCamera.transform.position - worldPosition).normalized;
            float facing = Vector3.Dot(frame.normal, viewDirection);

            if (facing <= frontFacingThreshold)
            {
                continue;
            }

            Vector2 particleScreenPosition = new Vector2(screenPosition.x, screenPosition.y);
            float distance = Vector2.Distance(mousePosition, particleScreenPosition);

            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearestParticle = i;
            }
        }

        return nearestParticle;
    }
}