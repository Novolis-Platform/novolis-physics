using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Abstractions;

/// <summary>Query-only static geometry: raycasts and approximate swept primitives.</summary>
public interface IStaticWorld
{
    /// <summary>Casts a ray up to <paramref name="maxDistance"/> and returns the closest hit.</summary>
    bool Raycast(in Ray ray, double maxDistance, out HitInfo hit);

    /// <summary>Approximate swept sphere vs static mesh (radius-inflated raycast; corners may be wrong).</summary>
    bool SweepSphere(in Sphere sphere, Vector3 displacement, out HitInfo hit);

    /// <summary>Conservative capsule sweep (samples segment endpoints as spheres).</summary>
    bool SweepCapsule(in Capsule capsule, Vector3 displacement, out HitInfo hit);
}
