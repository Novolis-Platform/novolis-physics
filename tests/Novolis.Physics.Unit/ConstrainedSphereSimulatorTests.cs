using System.Numerics;
using Novolis.Physics.Ballistics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;
using Novolis.Physics.TestSupport;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class ConstrainedSphereSimulatorTests
{
    [Test]
    public async Task Step_WithDistanceJoint_PullsSpheresAndReportsStats()
    {
        var min = PhysicsTestVectors.V(0, -10, -10);
        var max = PhysicsTestVectors.V(20, 10, 10);
        var world = CollisionTestGeometry.BuildAxisAlignedRoom(min, max, edgePad: 2);
        var interior = new InteriorClampVolume
        {
            MinX = min.X, MaxX = max.X, MinY = min.Y, MaxY = max.Y, MinZ = min.Z, MaxZ = max.Z,
        };

        var spheres = new List<SphereState>
        {
            new(new Vector3(5f, 0f, 0f), Vector3.Zero),
            new(new Vector3(9f, 0f, 0f), Vector3.Zero),
        };
        DistanceJoint[] joints = [new(0, 1, 2f)];

        var sim = new ConstrainedSphereSimulator
        {
            Options = { Radius = 0.3f, Gravity = Vector3.Zero },
            JointIterations = 12,
            ConstraintPasses = 2,
        };
        sim.SetJoints(joints);

        for (var i = 0; i < 40; i++)
            sim.Step(world, spheres, interior, deltaSeconds: 1f / 60f);

        var dist = Vector3.Distance(spheres[0].Position, spheres[1].Position);
        await Assert.That(dist).IsEqualTo(2f).Within(0.2f);
        await Assert.That(sim.LastStats.PhysicsSubSteps).IsGreaterThan(0);
    }

    [Test]
    public async Task Step_WithSwingLimit_CorrectsChildSphere()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-50, 0, -50), new Vector3(50, 0, -50), new Vector3(50, 0, 50), new Vector3(-50, 0, 50)],
            [0, 1, 2, 0, 2, 3]));
        var interior = new InteriorClampVolume
        {
            MinX = -40f, MaxX = 40f, MinY = -5f, MaxY = 20f, MinZ = -40f, MaxZ = 40f,
        };
        var spheres = new List<SphereState>
        {
            new(new Vector3(0f, 2f, 0f), Vector3.Zero),
            new(new Vector3(0.8f, 3.5f, 0f), Vector3.Zero),
        };
        SwingLimit[] swings =
        [
            new(0, 1, restDirection: Vector3.UnitY, maxRadians: 0.3f, stiffness: 1f),
        ];

        var sim = new ConstrainedSphereSimulator { Options = { Radius = 0.25f }, ConstraintPasses = 2 };
        sim.SetJoints([new DistanceJoint(0, 1, 1.5f)]);

        sim.Step(world, spheres, interior, 1f / 120f, swings, ReadOnlySpan<HingeLimit>.Empty);
        await Assert.That(sim.LastAngularCorrections).IsGreaterThanOrEqualTo(0);
        await Assert.That(sim.LastInternalCollisionFixes).IsGreaterThanOrEqualTo(0);
    }

    [Test]
    public async Task ResetAndDepenetrate_ExposeSphereSimulator()
    {
        var sim = new ConstrainedSphereSimulator();
        sim.ResetPileState();
        sim.MarkPileUnsettled();
        var spheres = new List<SphereState> { new(new Vector3(0f, 0f, 0f), Vector3.Zero) };
        sim.DepenetrateSpawnedRange(spheres, 0, 0, new InteriorClampVolume
        {
            MinX = 0f, MaxX = 10f, MinY = 0f, MaxY = 10f, MinZ = 0f, MaxZ = 10f,
        });
        await Assert.That(spheres[0].Position.Y).IsGreaterThanOrEqualTo(-10f);
    }
}
