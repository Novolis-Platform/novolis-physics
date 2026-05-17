using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Collision.Simple;

/// <summary>No-op <see cref="IStaticWorld"/> that never reports hits (tests and DI defaults).</summary>
public sealed class EmptyStaticWorld : IStaticWorld
{
    public bool Raycast(in Ray3 ray, double maxDistance, out HitInfo hit)
    {
        hit = default;
        return false;
    }

    public bool SweepSphere(in Sphere3 sphere, Vector3 displacement, out HitInfo hit)
    {
        hit = default;
        return false;
    }

    public bool SweepCapsule(in Capsule capsule, Vector3 displacement, out HitInfo hit)
    {
        hit = default;
        return false;
    }
}
