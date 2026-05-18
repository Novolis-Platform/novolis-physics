using System.Numerics;

namespace Novolis.Physics.Joints;

/// <summary>Ball-socket style limit: child direction from parent stays within a cone around <see cref="RestDirection"/>.</summary>
public readonly struct SwingLimit(
    int parentSphere,
    int childSphere,
    Vector3 restDirection,
    float maxRadians,
    float stiffness = 1f)
{
    public int ParentSphere { get; } = parentSphere;
    public int ChildSphere { get; } = childSphere;
    public Vector3 RestDirection { get; } = restDirection;
    public float MaxRadians { get; } = maxRadians;
    public float Stiffness { get; } = stiffness;
}
