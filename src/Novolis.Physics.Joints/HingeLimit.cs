using System.Numerics;

namespace Novolis.Physics.Joints;

/// <summary>Hinge limit in the plane perpendicular to <see cref="HingeAxis"/> (signed angle from <see cref="RestDirection"/>).</summary>
public readonly struct HingeLimit(
    int parentSphere,
    int childSphere,
    Vector3 hingeAxis,
    Vector3 restDirection,
    float minRadians,
    float maxRadians,
    float stiffness = 1f)
{
    public int ParentSphere { get; } = parentSphere;
    public int ChildSphere { get; } = childSphere;
    public Vector3 HingeAxis { get; } = hingeAxis;
    public Vector3 RestDirection { get; } = restDirection;
    public float MinRadians { get; } = minRadians;
    public float MaxRadians { get; } = maxRadians;
    public float Stiffness { get; } = stiffness;
}
