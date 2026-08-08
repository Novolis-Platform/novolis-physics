using Novolis.Physics.Abstractions;
using Novolis.Physics.Ballistics;
using Novolis.Physics.Collision.Simple;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class ProjectileTerrainStepperExtendedTests
{
    [Test]
    public async Task AdvanceOne_LongStep_SplitsSubsteps()
    {
        var terrain = new RangeTerrain(extent: 500f, groundY: 0f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(0, 0);
        var state = new ProjectileState(new Vector3(0, 50, 0), new Vector3(200, -10, 0), 1, 0);
        var options = new ProjectileTerrainStepOptions
        {
            DtSeconds = 1.0,
            MaxSweepMeters = 5f,
            ProjectileRadius = 0.1f,
        };

        var hit = ProjectileTerrainStepper.AdvanceOne(
            ref state,
            sim,
            env,
            collisionWorld: null,
            terrain,
            options,
            Vector3.Zero,
            out var impact);

        await Assert.That(hit || state.TimeSeconds > 0).IsTrue();
        if (hit)
            await Assert.That(impact).IsNotNull();
    }

    [Test]
    public async Task AdvanceOne_FallbackContact_OutsideRangeAfterStep()
    {
        var terrain = new RangeTerrain(extent: 30f, groundY: 0f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(0, 0);
        var state = new ProjectileState(new Vector3(25, 5, 25), new Vector3(80, 0, 80), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.2, ProjectileRadius = 0.05f, MaxSweepMeters = 50f };

        var hit = ProjectileTerrainStepper.AdvanceOne(
            ref state, sim, env, null, terrain, options, Vector3.Zero, out var impact);

        await Assert.That(hit).IsTrue();
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.BeyondRange);
    }

    [Test]
    public async Task AdvanceOne_MeshCollisionDuringChunkedSweep()
    {
        var verts = new[]
        {
            new Vector3(0, 0, 0), new Vector3(100, 0, 0), new Vector3(0, 0, 100),
        };
        var world = new BvhStaticWorld(new TriangleMesh(verts, [0, 1, 2]));
        var terrain = new RangeTerrain(extent: 200f, groundY: -100f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var state = new ProjectileState(new Vector3(10, 30, 10), new Vector3(0, -40, 0), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.5, MaxSweepMeters = 2f, ProjectileRadius = 0.2f };

        ProjectileTerrainImpact? impact = null;
        for (var i = 0; i < 40 && impact is null; i++)
        {
            ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, world, terrain, options, Vector3.Zero, out impact);
        }

        await Assert.That(impact).IsNotNull();
        await Assert.That(impact!.Value.Reason).IsEqualTo(ProjectileTerrainImpactReason.TerrainMesh);
    }

    [Test]
    public async Task AdvanceOne_SubstepSplit_MeshHitOnSubstep()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(0, 0, 0), new Vector3(200, 0, 0), new Vector3(0, 0, 200)],
            [0, 1, 2]));
        var terrain = new RangeTerrain(extent: 300f, groundY: -200f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var state = new ProjectileState(new Vector3(50, 120, 50), new Vector3(0, -250, 0), 0, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.35, MaxSweepMeters = 4f, ProjectileRadius = 0.15f };

        ProjectileTerrainImpact? impact = null;
        for (var i = 0; i < 8 && impact is null; i++)
            ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, world, terrain, options, Vector3.Zero, out impact);

        await Assert.That(impact).IsNotNull();
        await Assert.That(impact!.Value.Reason).IsEqualTo(ProjectileTerrainImpactReason.TerrainMesh);
    }

    [Test]
    public async Task AdvanceOne_TryTerrainHitLoop_HeightfieldMidChunk()
    {
        var terrain = new RangeTerrain(extent: 400f, groundY: 0f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var state = new ProjectileState(new Vector3(80, 60, 80), new Vector3(0, -90, 0), 0, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.6, MaxSweepMeters = 2.5f, ProjectileRadius = 0.12f };

        ProjectileTerrainImpact? impact = null;
        for (var i = 0; i < 12 && impact is null; i++)
            ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out impact);

        await Assert.That(impact).IsNotNull();
        await Assert.That(impact!.Value.Reason).IsEqualTo(ProjectileTerrainImpactReason.Heightfield);
    }

    [Test]
    public async Task AdvanceOne_NearZeroTravel_NoImpact()
    {
        var terrain = new RangeTerrain(extent: 100f, groundY: -10f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(0, 0);
        var state = new ProjectileState(new Vector3(10, 50, 10), new Vector3(0, 0, 0), 0, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 1e-9, MaxSweepMeters = 1f, ProjectileRadius = 0.1f };

        var hit = ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out var impact);

        await Assert.That(hit).IsFalse();
        await Assert.That(impact).IsNull();
    }

    [Test]
    public async Task AdvanceOne_BoundaryHitDuringChunkedTerrainSweep()
    {
        var terrain = new RangeTerrain(extent: 60f, groundY: -100f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(0, 0);
        var state = new ProjectileState(new Vector3(40, 20, 40), new Vector3(80, 0, 80), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.5, MaxSweepMeters = 2f, ProjectileRadius = 0.1f };

        var hit = ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out var impact);

        await Assert.That(hit).IsTrue();
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.BeyondRange);
    }

    private sealed class RangeTerrain(float extent, float groundY) : IProjectileTerrainContact
    {
        private readonly AxisAlignedRangeBox _box = new(extent);

        public bool IsInside(float x, float z) => _box.IsInside(x, z);

        public bool TryHeightfieldContact(Vector3 position, float radius) =>
            position.Y <= groundY + radius;

        public Vector3 ProjectOntoSurface(Vector3 position, float surfaceEpsilon = 0.05f) =>
            new(
                System.Math.Clamp(position.X, 0f, extent),
                groundY + surfaceEpsilon,
                System.Math.Clamp(position.Z, 0f, extent));

        public bool TrySegmentLeavesRange(Vector3 from, Vector3 to, out Vector3 hitPoint, out float fractionAlongSegment) =>
            _box.TrySegmentLeavesRange(from, to, out hitPoint, out fractionAlongSegment);
    }
}
