using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Joints;

/// <summary>Self-collision between spheres in a ragdoll group (prevents folding through itself).</summary>
public static class RagdollBodyCollision
{
    public static int ResolveOverlaps(
        IList<SphereState> spheres,
        float radius,
        int iterations = 4,
        float separationScale = 1.02f)
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
}
