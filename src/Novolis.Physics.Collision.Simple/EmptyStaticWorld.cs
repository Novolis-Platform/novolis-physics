using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Collision.Simple;

/// <summary>No-op <see cref="IStaticWorld"/> that never reports hits (tests and DI defaults).</summary>
public sealed class EmptyStaticWorld : IStaticWorld
{
    /// <inheritdoc />
    public bool Raycast(in Ray ray, double maxDistance, out HitInfo hit)
    {
        hit = default;
        return false;
    }

    /// <inheritdoc />
    public bool SweepSphere(in Sphere sphere, Vector3 displacement, out HitInfo hit)
    {
        hit = default;
        return false;
    }

    /// <inheritdoc />
    public bool SweepCapsule(in Capsule capsule, Vector3 displacement, out HitInfo hit)
    {
        hit = default;
        return false;
    }
}
