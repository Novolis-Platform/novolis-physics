namespace Novolis.Physics.Joints;

/// <summary>Fixed rest length between two sphere indices in a shared <see cref="Collision.Simple.SphereState"/> list.</summary>
public readonly struct DistanceJoint(int sphereA, int sphereB, float restLength)
{
    public int SphereA { get; } = sphereA;
    public int SphereB { get; } = sphereB;
    public float RestLength { get; } = restLength;
}
