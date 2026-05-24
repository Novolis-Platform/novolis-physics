using System.Numerics;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Position and velocity resolution for equal-radius sphere pairs.</summary>
public static class SphereOverlapResolution
{
    /// <summary>Separate operation.</summary>
    public static bool Separate(
        ref Vector3 positionA,
        ref Vector3 positionB,
        float radius,
        float separationScale = 1.001f)
    {
        var minDist = radius * 2f * separationScale;
        var minDistSq = minDist * minDist;
        var delta = positionB - positionA;
        var distSq = delta.LengthSquared();
        if (distSq >= minDistSq)
            return false;

        Vector3 normal;
        float overlap;
        if (distSq < 1e-10f)
        {
            normal = new Vector3(1f, 0f, 0f);
            overlap = minDist;
        }
        else
        {
            var dist = MathF.Sqrt(distSq);
            normal = delta / dist;
            overlap = minDist - dist;
        }

        positionA -= normal * (overlap * 0.5f);
        positionB += normal * (overlap * 0.5f);
        return true;
    }
/// <summary>SeparateWithImpulse operation.</summary>

    public static bool SeparateWithImpulse(
        ref Vector3 positionA,
        ref Vector3 positionB,
        ref Vector3 velocityA,
        ref Vector3 velocityB,
        float radius,
        float restitution,
        float separationScale = 1.001f)
    {
        if (!Separate(ref positionA, ref positionB, radius, separationScale))
            return false;

        var delta = positionB - positionA;
        var distSq = delta.LengthSquared();
        Vector3 normal;
        if (distSq < 1e-10f)
            normal = new Vector3(1f, 0f, 0f);
        else
            normal = delta / MathF.Sqrt(distSq);

        var relVel = velocityB - velocityA;
        var vn = Vector3.Dot(relVel, normal);
        if (vn >= 0f)
            return true;

        var impulse = -(1f + restitution) * vn * 0.5f;
        velocityA -= normal * impulse;
        velocityB += normal * impulse;
        return true;
    }
}
