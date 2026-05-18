namespace Novolis.Physics.Joints;

/// <summary>Fixed rest length between two sphere indices in a shared <see cref="Collision.Simple.SphereState"/> list.</summary>
public readonly struct DistanceJoint(int sphereA, int sphereB, float restLength, float stiffness = 1f)
{
    public int SphereA { get; } = sphereA;
    public int SphereB { get; } = sphereB;
    public float RestLength { get; } = restLength;
    /// <summary>Constraint strength in 0..1 (1 = rigid, lower = softer).</summary>
    public float Stiffness { get; } = stiffness;
}
