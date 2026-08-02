using System.Numerics;
using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Joints;

/// <summary>Iterative position projection for distance constraints (equal-mass spheres).</summary>
public static class DistanceJointSolver
{
    /// <summary>
    /// Projects distance constraints.
    /// </summary>
    /// <param name="joints">Distance joints to satisfy.</param>
    /// <param name="spheres">Shared particle states.</param>
    /// <param name="iterations">Solver iterations.</param>
    /// <param name="positionSlop">Ignore errors smaller than this (meters).</param>
    /// <param name="maxStrainFraction">
    /// Per-iteration correction cap as a fraction of rest length (ragdoll default 0.35).
    /// Cloth should pass a large value (e.g. 2+) or <see cref="float.PositiveInfinity"/> so fabric does not go doughy under gravity.
    /// </param>
    public static int Solve(
        ReadOnlySpan<DistanceJoint> joints,
        IList<SphereState> spheres,
        int iterations = 8,
        float positionSlop = 1e-4f,
        float maxStrainFraction = 0.35f)
    {
        if (joints.Length == 0 || spheres.Count == 0 || iterations <= 0)
            return 0;

        var strainCap = maxStrainFraction;
        if (float.IsNaN(strainCap) || strainCap <= 0f)
            strainCap = 0.35f;

        var corrections = 0;
        for (var iter = 0; iter < iterations; iter++)
        {
            foreach (var joint in joints)
            {
                if ((uint)joint.SphereA >= (uint)spheres.Count || (uint)joint.SphereB >= (uint)spheres.Count)
                    continue;

                var a = spheres[joint.SphereA];
                var b = spheres[joint.SphereB];
                var delta = b.Position - a.Position;
                var distSq = delta.LengthSquared();
                if (distSq < 1e-10f)
                    continue;

                var dist = MathF.Sqrt(distSq);
                var error = dist - joint.RestLength;
                if (MathF.Abs(error) <= positionSlop)
                    continue;

                var n = delta / dist;
                var strength = System.Math.Clamp(joint.Stiffness, 0f, 1f);
                if (!float.IsPositiveInfinity(strainCap))
                {
                    var maxError = joint.RestLength * strainCap;
                    error = System.Math.Clamp(error, -maxError, maxError);
                }

                var correction = n * (error * 0.5f * strength);
                a.Position += correction;
                b.Position -= correction;
                spheres[joint.SphereA] = a;
                spheres[joint.SphereB] = b;
                corrections++;
            }
        }

        return corrections;
    }
}
