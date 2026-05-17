using Novolis.Physics.Abstractions;
using Novolis.Physics.Ballistics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.TestSupport;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class ProjectileTerrainStepperTests
{
    [Test]
    public async Task AdvanceOne_HitsFlatMeshTriangle()
    {
        var verts = new[]
        {
            PhysicsTestVectors.V(0, 0, 0),
            PhysicsTestVectors.V(50, 0, 0),
            PhysicsTestVectors.V(0, 0, 50),
        };
        var world = new BvhStaticWorld(new StaticTriangleMesh(verts, new[] { 0, 1, 2 }));
        var terrain = new FlatTerrainContact(heightfieldAtZero: false);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var runner = new BallisticTrajectoryRunner(new BallisticTrajectoryRunnerOptions
        {
            Step = new ProjectileTerrainStepOptions { DtSeconds = 1.0 / 120.0, ProjectileRadius = 0.15f },
        });
        runner.Begin(new ProjectileState(PhysicsTestVectors.V(5, 6, 5), PhysicsTestVectors.V(0, -25, 0), 1, 0));

        ProjectileTerrainImpact? impact = null;
        for (var i = 0; i < 500 && runner.Phase == BallisticTrajectoryPhase.InFlight; i++)
        {
            runner.AdvanceOne(sim, env, world, terrain, Vector3.Zero);
            impact = runner.Impact;
        }

        await Assert.That(runner.Phase).IsEqualTo(BallisticTrajectoryPhase.Impacted);
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.TerrainMesh);
    }

    [Test]
    public async Task AdvanceOne_RangeExit_ProjectsOntoGround()
    {
        var terrain = new FlatTerrainContact(heightfieldAtZero: true, extent: 100f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var state = new ProjectileState(new Vector3(90, 8, 50), new Vector3(120, 0, 0), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.1, ProjectileRadius = 0.08f };

        var hit = ProjectileTerrainStepper.AdvanceOne(
            ref state,
            sim,
            env,
            collisionWorld: null,
            terrain,
            options,
            Vector3.Zero,
            out var impact);

        await Assert.That(hit).IsTrue();
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.BeyondRange);
        await Assert.That(impact!.Value.Position.Y).IsLessThan(55f);
        await Assert.That(impact.Value.Position.X).IsEqualTo(100f).Within(0.5f);
    }

    [Test]
    public async Task BallisticTrajectoryRunner_RecordsTrailAndImpact()
    {
        var terrain = new FlatTerrainContact(heightfieldAtZero: true);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var runner = new BallisticTrajectoryRunner();
        var start = new ProjectileState(new Vector3(0, 10, 0), new Vector3(0, -15, 0), 1, 0);
        runner.Begin(start);

        for (var i = 0; i < 200 && runner.Phase == BallisticTrajectoryPhase.InFlight; i++)
            runner.AdvanceOne(sim, env, collisionWorld: null, terrain, Vector3.Zero);

        await Assert.That(runner.Phase).IsEqualTo(BallisticTrajectoryPhase.Impacted);
        await Assert.That(runner.Trail.Count).IsGreaterThan(1);
        await Assert.That(runner.Impact).IsNotNull();
    }

    private sealed class FlatTerrainContact(bool heightfieldAtZero, float extent = 200f) : IProjectileTerrainContact
    {
        private readonly AxisAlignedRangeBox _box = new(extent);

        public bool IsInside(float x, float z) => _box.IsInside(x, z);

        public bool TryHeightfieldContact(Vector3 position, float radius) =>
            heightfieldAtZero && position.Y <= radius;

        public Vector3 ProjectOntoSurface(Vector3 position, float surfaceEpsilon = 0.05f) =>
            new(
                System.Math.Clamp(position.X, 0f, extent),
                surfaceEpsilon,
                System.Math.Clamp(position.Z, 0f, extent));

        public bool TrySegmentLeavesRange(Vector3 from, Vector3 to, out Vector3 hitPoint, out float fractionAlongSegment) =>
            _box.TrySegmentLeavesRange(from, to, out hitPoint, out fractionAlongSegment);
    }
}
