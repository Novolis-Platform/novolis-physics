using Novolis.Physics.TestSupport;
using Novolis.Physics.Collision.Simple;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

/// <summary>
/// Documents approximate <see cref="BvhStaticWorld.SweepSphere"/> behavior (see INTEGRATION.md §3).
/// </summary>
[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class SweepLimitationScenarioTests
{
    [Test]
    public async Task SweepSphere_LargeStepOvershoot_MissesWhileSubStepsHit()
    {
        var verts = new[]
        {
            PhysicsTestVectors.V(0, 0, 0),
            PhysicsTestVectors.V(10, 0, 0),
            PhysicsTestVectors.V(0, 0, 10),
        };
        var world = new BvhStaticWorld(new TriangleMesh(verts, new[] { 0, 1, 2 }));
        var sphere = new Sphere(PhysicsTestVectors.V(1, (float)5.0, 1), radius: 0.15f);
        var largeDisplacement = PhysicsTestVectors.V(0, -4, 0);

        var largeHit = world.SweepSphere(in sphere, largeDisplacement, out _);

        var subStep = PhysicsTestVectors.V(0, -0.2, 0);
        var probe = sphere;
        var anySubHit = false;
        for (var i = 0; i < 30 && !anySubHit; i++)
        {
            if (world.SweepSphere(in probe, subStep, out var subHit))
            {
                anySubHit = true;
                await Assert.That(subHit.Distance).IsGreaterThan((float)(0)).And.IsLessThan((float)(subStep.Length()));
            }
            else
            {
                probe = new Sphere(probe.Center + subStep, (float)probe.Radius);
            }
        }

        await Assert.That(largeHit).IsFalse();
        await Assert.That(anySubHit).IsTrue();
    }
}
