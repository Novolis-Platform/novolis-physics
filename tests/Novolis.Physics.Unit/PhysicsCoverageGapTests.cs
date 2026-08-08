using Novolis.Physics.Abstractions;
using Novolis.Physics.Ballistics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class PhysicsCoverageGapTests
{
    [Test]
    public async Task ProjectileTerrainStepper_HeightfieldHit_OnShortStep()
    {
        var terrain = new GroundTerrain(groundY: 0f, extent: 200f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var state = new ProjectileState(new Vector3(20, 0.04f, 20), new Vector3(0, -5, 0), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.05, ProjectileRadius = 0.05f, MaxSweepMeters = 50f };

        var hit = ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out var impact);

        await Assert.That(hit).IsTrue();
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.Heightfield);
    }

    [Test]
    public async Task ProjectileTerrainStepper_FallbackHeightfield_AfterIntegration()
    {
        var terrain = new GroundTerrain(groundY: 0f, extent: 200f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(0, 0);
        var state = new ProjectileState(new Vector3(10, 0.02f, 10), new Vector3(0, 0, 0), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.01, ProjectileRadius = 0.05f, MaxSweepMeters = 50f };

        var hit = ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out var impact);

        await Assert.That(hit).IsTrue();
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.Heightfield);
    }

    [Test]
    public async Task ProjectileTerrainStepper_ChunkedSweep_HitsHeightfieldMidPath()
    {
        var terrain = new GroundTerrain(groundY: 0f, extent: 500f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var state = new ProjectileState(new Vector3(50, 40, 50), new Vector3(0, -120, 0), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 1.0, MaxSweepMeters = 3f, ProjectileRadius = 0.1f };

        var hit = ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out var impact);

        await Assert.That(hit).IsTrue();
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.Heightfield);
    }

    [Test]
    public async Task SphereSimulator_EarlyExit_WhenDeltaNonPositive()
    {
        var sim = new SphereInStaticWorldSimulator();
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var spheres = new List<SphereState> { new(Vector3.Zero, Vector3.Zero) };
        var interior = new InteriorClampVolume { MinX = -1, MaxX = 1, MinY = 0, MaxY = 1, MinZ = -1, MaxZ = 1 };
        sim.Step(world, spheres, interior, 0f);
        await Assert.That(sim.LastStats.ActiveCount).IsEqualTo(0);
    }

    [Test]
    public async Task SphereSimulator_ClampVelocityAndInterior()
    {
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var interior = new InteriorClampVolume { MinX = -1, MaxX = 1, MinY = 0, MaxY = 2, MinZ = -1, MaxZ = 1 };
        var spheres = new List<SphereState>
        {
            new(new Vector3(5, 0.3f, 0), new Vector3(100, 5, 0)),
        };
        var sim = new SphereInStaticWorldSimulator
        {
            Options = { Radius = 0.25f, FloorHeight = 0f, MaxSpeedMps = 2f, Gravity = Vector3.Zero },
        };
        sim.Step(world, spheres, interior, 1f / 60f);
        await Assert.That(spheres[0].Velocity.Length()).IsLessThanOrEqualTo(2.1f);
        await Assert.That(sim.LastStats.ClampedCount).IsGreaterThan(0);
    }

    [Test]
    public async Task SphereSimulator_DepenetratePartialRange()
    {
        var sim = new SphereInStaticWorldSimulator { Options = { Radius = 0.2f, MaxSpeedMps = 5f } };
        var spheres = Enumerable.Range(0, 20)
            .Select(i => new SphereState(new Vector3(i * 0.05f, 1, 0), Vector3.Zero))
            .ToList();
        var interior = new InteriorClampVolume { MinX = -5, MaxX = 5, MinY = 0, MaxY = 5, MinZ = -5, MaxZ = 5 };
        sim.DepenetrateSpawnedRange(spheres, 5, 15, interior);
        await Assert.That(Vector3.Distance(spheres[5].Position, spheres[6].Position)).IsGreaterThan(0.35f);
    }

    [Test]
    public async Task SphereSimulator_MarkPileUnsettled_ResetsSkip()
    {
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var interior = new InteriorClampVolume { MinX = -1, MaxX = 1, MinY = 0, MaxY = 1, MinZ = -1, MaxZ = 1 };
        var spheres = new List<SphereState>
        {
            new(new Vector3(0, 0.3f, 0), Vector3.Zero) { IsSleeping = true, IsGrounded = true },
            new(new Vector3(0.5f, 0.3f, 0), Vector3.Zero) { IsSleeping = true, IsGrounded = true },
        };
        var sim = new SphereInStaticWorldSimulator { Options = { Radius = 0.2f, FloorHeight = 0f } };
        for (var i = 0; i < 20; i++)
            sim.Step(world, spheres, interior, 1f / 60f);
        sim.MarkPileUnsettled();
        sim.Step(world, spheres, interior, 1f / 60f);
        await Assert.That(sim.LastStats.SphereContactSkipped).IsFalse();
    }

    [Test]
    public async Task BvhStaticWorld_SweepSphere_ZeroDisplacement_ReturnsFalse()
    {
        var world = new BvhStaticWorld(new TriangleMesh([Vector3.Zero, Vector3.UnitX, Vector3.UnitY], [0, 1, 2]));
        await Assert.That(world.SweepSphere(new Sphere(Vector3.UnitY, 0.1f), Vector3.Zero, out _)).IsFalse();
    }

    [Test]
    public async Task BvhStaticWorld_SweepSphere_MissWhenHitBeyondTravel()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-10, 0, -10), new Vector3(10, 0, -10), new Vector3(-10, 0, 10)],
            [0, 1, 2]));
        var sphere = new Sphere(new Vector3(0, 10, 0), 0.1f);
        await Assert.That(world.SweepSphere(in sphere, new Vector3(0, -1, 0), out _)).IsFalse();
    }

    [Test]
    public async Task BvhStaticWorld_SweepCapsule_OnlyLowerEndpointHits()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-20, 0, -20), new Vector3(20, 0, -20), new Vector3(-20, 0, 20), new Vector3(20, 0, 20)],
            [0, 1, 2, 1, 3, 2]));
        var cap = new Capsule(new Vector3(0, 8, 0), new Vector3(0, 0.45f, 0), radius: 0.35f);
        var hit = world.SweepCapsule(in cap, new Vector3(0, -0.3f, 0), out var info);
        await Assert.That(hit).IsTrue();
        await Assert.That(info.Distance).IsGreaterThan(0);
    }

    [Test]
    public async Task AngularLimitSolver_HingeWithFrameReference_Corrects()
    {
        var root = new Vector3(0f, 0f, 0f);
        var mid = new Vector3(0f, 1f, 0f);
        var tip = new Vector3(-2f, 1f, 1.5f);
        var spheres = new List<SphereState>
        {
            new(root, Vector3.Zero),
            new(mid, Vector3.Zero),
            new(tip, Vector3.Zero),
        };
        HingeLimit[] limits =
        [
            HingeLimit.CreateLocal(1, 2, frameReferenceSphere: 0, Vector3.UnitY, Vector3.UnitX, minRadians: -0.15f, maxRadians: 0.15f),
        ];
        var corrections = AngularLimitSolver.Solve(ReadOnlySpan<SwingLimit>.Empty, limits, spheres, iterations: 12);
        await Assert.That(corrections).IsGreaterThan(0);
    }

    [Test]
    public async Task AngularLimitSolver_SwingZeroStiffness_NoCorrection()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 0.5f, 1.5f), Vector3.Zero),
        };
        SwingLimit[] limits = [new(0, 1, Vector3.UnitY, maxRadians: 0.2f, stiffness: 0f)];
        var corrections = AngularLimitSolver.Solve(limits, ReadOnlySpan<HingeLimit>.Empty, spheres, iterations: 4);
        await Assert.That(corrections).IsEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_HingeParallelToAxis_UsesRestPlaneFallback()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 1f, 0f), Vector3.Zero),
        };
        HingeLimit[] limits =
        [
            new(0, 1, Vector3.UnitY, Vector3.UnitX, minRadians: -0.5f, maxRadians: 0.5f, stiffness: 1f),
        ];
        var corrections = AngularLimitSolver.Solve(ReadOnlySpan<SwingLimit>.Empty, limits, spheres, iterations: 6);
        await Assert.That(corrections).IsGreaterThanOrEqualTo(0);
    }

    private sealed class GroundTerrain(float groundY, float extent) : IProjectileTerrainContact
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
