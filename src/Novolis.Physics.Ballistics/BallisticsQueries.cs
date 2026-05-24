using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Ballistics;

/// <summary>Raycast and sweep helpers over <see cref="IStaticWorld"/> for projectile-sized queries.</summary>
public static class BallisticsQueries
{
    /// <summary>LineOfSight operation.</summary>
    public static bool LineOfSight(IStaticWorld world, in Ray3 ray, double maxDistance, out HitInfo hit) =>
        world.Raycast(in ray, maxDistance, out hit);
/// <summary>SweepProjectileSphere operation.</summary>

    public static bool SweepProjectileSphere(
        IStaticWorld world,
        in Sphere3 sphere,
        Vector3 displacement,
        out HitInfo hit) =>
        world.SweepSphere(in sphere, displacement, out hit);
}
