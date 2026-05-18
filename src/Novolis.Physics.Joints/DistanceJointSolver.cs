using System.Numerics;
using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Joints;

/// <summary>Iterative position projection for distance constraints (equal-mass spheres).</summary>
public static class DistanceJointSolver
{
    public static int Solve(
        ReadOnlySpan<DistanceJoint> joints,
        IList<SphereState> spheres,
        int iterations = 8,
        float positionSlop = 1e-4f)
    {
        if (joints.Length == 0 || spheres.Count == 0 || iterations <= 0)
            return 0;

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
                var correction = n * (error * 0.5f * strength);
                a.Position += correction;
                b.Position -= correction;
                corrections++;
            }
        }

        return corrections;
    }
}
