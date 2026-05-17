using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Collision.Simple;

/// <summary>
/// Kinematic updates for a point mass contacting a fixed rigid surface. Works in any inertial frame
/// where the surface is immovable (planet surface patch, station bulkhead, etc.).
/// </summary>
public static class SphereContactKinematics
{
    /// <summary>
    /// Newton restitution against a fixed plane: <c>v' = v − (1+e)(v·n)n</c>.
    /// <paramref name="unitNormal"/> must match <see cref="Novolis.Physics.Abstractions.HitInfo.Normal"/> from
    /// <see cref="BvhStaticWorld.SweepSphere"/> (unit vector from the collision query).
    /// </summary>
    public static Vector3 ReflectWithRestitution(
        in Vector3 velocityMps,
        in Vector3 unitNormal,
        double coefficientOfRestitution)
    {
        var e = System.Math.Clamp(coefficientOfRestitution, 0.0, 1.0);
        var vn = Vector3.Dot(velocityMps, unitNormal);
        return velocityMps - unitNormal.Multiply((1.0 + e) * vn);
    }

    /// <summary>Perfectly elastic reflection (<c>e = 1</c>).</summary>
    public static Vector3 ReflectElastic(in Vector3 velocityMps, in Vector3 unitNormal) =>
        ReflectWithRestitution(velocityMps, unitNormal, 1.0);
}
