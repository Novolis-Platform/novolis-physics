using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Joints;

/// <summary>Self-collision between spheres in a ragdoll group (prevents folding through itself).</summary>
public static class RagdollBodyCollision
{
    /// <summary>Separates overlapping spheres for <paramref name="iterations"/> passes.</summary>
    public static int ResolveOverlaps(
        IList<SphereState> spheres,
        float radius,
        int iterations = 4,
        float separationScale = 1.02f) =>
        ResolveOverlaps(spheres, radius, iterations, separationScale, ReadOnlySpan<(int, int)>.Empty);

    /// <summary>Separates overlaps while skipping adjacent joint pairs in <paramref name="skipPairs"/>.</summary>
    public static int ResolveOverlaps(
        IList<SphereState> spheres,
        float radius,
        int iterations,
        float separationScale,
        ReadOnlySpan<(int A, int B)> skipPairs)
    {
        if (spheres.Count < 2 || iterations <= 0)
            return 0;

        var fixes = 0;
        for (var pass = 0; pass < iterations; pass++)
        {
            for (var i = 0; i < spheres.Count; i++)
            {
                for (var j = i + 1; j < spheres.Count; j++)
                {
                    if (ShouldSkipPair(i, j, skipPairs))
                        continue;

                    var a = spheres[i];
                    var b = spheres[j];
                    if (SphereOverlapResolution.Separate(
                            ref a.Position,
                            ref b.Position,
                            radius,
                            separationScale))
                        fixes++;
                }
            }
        }

        return fixes;
    }

    /// <summary>Builds undirected sphere index pairs from distance joints (for skip lists).</summary>
    public static (int A, int B)[] BuildAdjacentSkipPairs(ReadOnlySpan<DistanceJoint> joints)
    {
        if (joints.Length == 0)
            return [];

        var pairs = new HashSet<(int, int)>();
        foreach (var joint in joints)
        {
            var a = joint.SphereA;
            var b = joint.SphereB;
            if (a > b)
                (a, b) = (b, a);
            pairs.Add((a, b));
        }

        var result = new (int, int)[pairs.Count];
        pairs.CopyTo(result);
        return result;
    }

    private static bool ShouldSkipPair(int i, int j, ReadOnlySpan<(int A, int B)> skipPairs)
    {
        if (skipPairs.Length == 0)
            return false;

        if (i > j)
            (i, j) = (j, i);

        foreach (var pair in skipPairs)
        {
            if (pair.A == i && pair.B == j)
                return true;
        }

        return false;
    }
}
