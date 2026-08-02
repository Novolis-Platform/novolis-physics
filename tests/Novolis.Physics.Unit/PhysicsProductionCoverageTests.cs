using Novolis.Physics.Abstractions;
using Novolis.Physics.Ballistics;
using Novolis.Physics.Cloth;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class PhysicsProductionCoverageTests
{
    [Test]
    public async Task ProjectileTerrainStepper_SubstepSplit_HitsGroundOnSubstep()
    {
        var terrain = new DelayedHeightfieldTerrain(extent: 500f, groundY: 0f, radius: 0.1f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var state = new ProjectileState(new Vector3(50, 130, 50), new Vector3(0, -135, 0), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 1.0, MaxSweepMeters = 3f, ProjectileRadius = 0.1f };

        var hit = ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out var impact);

        await Assert.That(hit).IsTrue();
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.Heightfield);
    }

    [Test]
    public async Task ProjectileTerrainStepper_Fallback_BeyondRangeAfterSubsteps()
    {
        var terrain = new NoSegmentExitTerrain(extent: 31f, groundY: -500f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(0, 0);
        var state = new ProjectileState(new Vector3(28, 5, 28), new Vector3(6, 0, 6), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.65, MaxSweepMeters = 4f, ProjectileRadius = 0.05f };

        var hit = ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out var impact);

        await Assert.That(hit).IsTrue();
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.BeyondRange);
    }

    [Test]
    public async Task ProjectileTerrainStepper_Fallback_HeightfieldAfterIntegration()
    {
        var terrain = new DelayedHeightfieldTerrain(extent: 200f, groundY: 0f, radius: 0.05f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(0, 0);
        var state = new ProjectileState(new Vector3(10, 0.04f, 10), new Vector3(0, 0, 0), 0, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.001, MaxSweepMeters = 50f, ProjectileRadius = 0.05f };

        var hit = ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out var impact);

        await Assert.That(hit).IsTrue();
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.Heightfield);
    }

    [Test]
    public async Task ProjectileTerrainStepper_ChunkedTerrainSweep_HitsBoundaryMidPath()
    {
        var terrain = new NoSegmentExitTerrain(extent: 45f, groundY: -200f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(0, 0);
        var state = new ProjectileState(new Vector3(38, 8, 38), new Vector3(12, 0, 12), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.7, MaxSweepMeters = 2f, ProjectileRadius = 0.05f };

        var hit = ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out var impact);

        await Assert.That(hit).IsTrue();
        await Assert.That(impact?.Reason).IsEqualTo(ProjectileTerrainImpactReason.BeyondRange);
    }

    [Test]
    public async Task ProjectileTerrainStepper_ChunkedTerrainSweep_HitsHeightfieldMidPath()
    {
        var terrain = new DelayedHeightfieldTerrain(extent: 400f, groundY: 0f, radius: 0.12f);
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var state = new ProjectileState(new Vector3(80, 60, 80), new Vector3(0, -90, 0), 1, 0);
        var options = new ProjectileTerrainStepOptions { DtSeconds = 0.6, MaxSweepMeters = 2.5f, ProjectileRadius = 0.12f };

        ProjectileTerrainImpact? impact = null;
        for (var i = 0; i < 12 && impact is null; i++)
            ProjectileTerrainStepper.AdvanceOne(ref state, sim, env, null, terrain, options, Vector3.Zero, out impact);

        await Assert.That(impact).IsNotNull();
        await Assert.That(impact!.Value.Reason).IsEqualTo(ProjectileTerrainImpactReason.Heightfield);
    }

    [Test]
    public async Task AngularLimitSolver_EmptyInput_ReturnsZero()
    {
        await Assert.That(AngularLimitSolver.Solve(ReadOnlySpan<SwingLimit>.Empty, ReadOnlySpan<HingeLimit>.Empty, [], iterations: 0)).IsEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_SwingCoincidentSpheres_NoCorrection()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(Vector3.Zero, Vector3.Zero),
        };
        SwingLimit[] limits = [new(0, 1, Vector3.UnitY, maxRadians: 0.2f, stiffness: 1f)];
        await Assert.That(AngularLimitSolver.SolveSwing(limits[0], spheres)).IsEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_HingeInvalidIndices_ReturnsZero()
    {
        var spheres = new List<SphereState> { new(Vector3.Zero, Vector3.Zero) };
        HingeLimit[] limits = [new(0, 3, Vector3.UnitY, Vector3.UnitX, -0.5f, 0.5f, stiffness: 1f)];
        await Assert.That(AngularLimitSolver.SolveHinge(limits[0], spheres)).IsEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_HingeCoincidentSpheres_ReturnsZero()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(Vector3.Zero, Vector3.Zero),
        };
        HingeLimit[] limits = [new(0, 1, Vector3.UnitY, Vector3.UnitX, -0.5f, 0.5f, stiffness: 1f)];
        await Assert.That(AngularLimitSolver.SolveHinge(limits[0], spheres)).IsEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_HingeRestParallelToAxis_ReturnsZero()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 0f, 1f), Vector3.Zero),
        };
        HingeLimit[] limits = [new(0, 1, Vector3.UnitY, Vector3.UnitY, -0.5f, 0.5f, stiffness: 1f)];
        await Assert.That(AngularLimitSolver.SolveHinge(limits[0], spheres)).IsEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_SwingFrameReferenceOutOfBounds_UsesWorldRest()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 0.5f, 1.5f), Vector3.Zero),
        };
        SwingLimit[] limits =
        [
            SwingLimit.CreateLocal(0, 1, frameReferenceSphere: 9, Vector3.UnitY, maxRadians: 0.25f, stiffness: 1f),
        ];
        var corrections = AngularLimitSolver.Solve(limits, ReadOnlySpan<HingeLimit>.Empty, spheres, iterations: 6);
        await Assert.That(corrections).IsGreaterThanOrEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_SwingFrameReferenceDegenerate_UsesWorldRest()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 0.5f, 1.5f), Vector3.Zero),
        };
        spheres[2] = new SphereState(new Vector3(0f, 0.5f, 1.8f), Vector3.Zero);
        SwingLimit[] limits =
        [
            SwingLimit.CreateLocal(0, 2, frameReferenceSphere: 1, Vector3.UnitY, maxRadians: 0.3f, stiffness: 1f),
        ];
        var corrections = AngularLimitSolver.Solve(limits, ReadOnlySpan<HingeLimit>.Empty, spheres, iterations: 8);
        await Assert.That(corrections).IsGreaterThanOrEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_HingeFrameReferenceOutOfBounds_UsesWorldAxes()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 1f, 0.5f), Vector3.Zero),
        };
        HingeLimit[] limits =
        [
            HingeLimit.CreateLocal(0, 1, frameReferenceSphere: 5, Vector3.UnitX, Vector3.UnitY, -0.4f, 0.4f, stiffness: 1f),
        ];
        var corrections = AngularLimitSolver.Solve(ReadOnlySpan<SwingLimit>.Empty, limits, spheres, iterations: 6);
        await Assert.That(corrections).IsGreaterThanOrEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_HingeFrameReferenceDegenerate_UsesWorldAxes()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 1f, 0.5f), Vector3.Zero),
        };
        HingeLimit[] limits =
        [
            HingeLimit.CreateLocal(0, 2, frameReferenceSphere: 1, Vector3.UnitX, Vector3.UnitY, -0.4f, 0.4f, stiffness: 1f),
        ];
        var corrections = AngularLimitSolver.Solve(ReadOnlySpan<SwingLimit>.Empty, limits, spheres, iterations: 6);
        await Assert.That(corrections).IsGreaterThanOrEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_HingeZeroRestDirection_ReturnsZero()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 1f, 0f), Vector3.Zero),
        };
        HingeLimit[] limits = [new(0, 1, Vector3.UnitX, Vector3.Zero, -0.5f, 0.5f, stiffness: 1f)];
        await Assert.That(AngularLimitSolver.SolveHinge(limits[0], spheres)).IsEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_SwingBoneParallelToRest_StillClamps()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 1.001f, 0f), Vector3.Zero),
        };
        SwingLimit[] limits = [new(0, 1, Vector3.UnitY, maxRadians: 0.01f, stiffness: 1f)];
        var corrections = AngularLimitSolver.Solve(limits, ReadOnlySpan<HingeLimit>.Empty, spheres, iterations: 4);
        await Assert.That(corrections).IsGreaterThanOrEqualTo(0);
    }

    [Test]
    public async Task BvhStaticWorld_SweepSphere_HitBeyondTravel_ReturnsFalse()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-10, 0, -10), new Vector3(10, 0, -10), new Vector3(-10, 0, 10)],
            [0, 1, 2]));
        var sphere = new Sphere(new Vector3(0, 5, 0), 0.1f);
        await Assert.That(world.SweepSphere(in sphere, new Vector3(0, -0.5f, 0), out _)).IsFalse();
    }

    [Test]
    public async Task BvhStaticWorld_SweepSphere_ShallowOverlap_AdjustsDistance()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(0, 0, 0), new Vector3(10, 0, 0), new Vector3(0, 0, 10)],
            [0, 1, 2]));
        var sphere = new Sphere(new Vector3(1, 0.42f, 1), 0.5f);
        var hit = world.SweepSphere(in sphere, new Vector3(0, -0.02f, 0), out var info);
        await Assert.That(hit).IsTrue();
        await Assert.That(info.Distance).IsGreaterThan(0);
    }

    [Test]
    public async Task BvhStaticWorld_SweepSphere_DeepOverlap_ReturnsFalse()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-10, 0, -10), new Vector3(10, 0, -10), new Vector3(-10, 0, 10)],
            [0, 1, 2]));
        var sphere = new Sphere(new Vector3(0, -0.6f, 0), 0.5f);
        await Assert.That(world.SweepSphere(in sphere, new Vector3(0, -0.01f, 0), out _)).IsFalse();
    }

    [Test]
    public async Task BvhStaticWorld_SweepCapsule_OnlyUpperEndpointHits()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-20, 0, -20), new Vector3(20, 0, -20), new Vector3(-20, 0, 20), new Vector3(20, 0, 20)],
            [0, 1, 2, 1, 3, 2]));
        var cap = new Capsule(new Vector3(0, 0.45f, 0), new Vector3(0, 8, 0), radius: 0.35f);
        var hit = world.SweepCapsule(in cap, new Vector3(0, -0.25f, 0), out var info);
        await Assert.That(hit).IsTrue();
        await Assert.That(info.Distance).IsGreaterThan(0);
    }

    [Test]
    public async Task BvhStaticWorld_SweepCapsule_NoEndpointHits_ReturnsFalse()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-20, 0, -20), new Vector3(20, 0, -20), new Vector3(-20, 0, 20)],
            [0, 1, 2]));
        var cap = new Capsule(new Vector3(0, 5, 0), new Vector3(0, 8, 0), radius: 0.2f);
        await Assert.That(world.SweepCapsule(in cap, new Vector3(1, 0, 0), out _)).IsFalse();
    }

    [Test]
    public async Task ClothSheetPreset_SmallGrid_ReturnsZeroTriangleCount()
    {
        await Assert.That(ClothSheetPreset.TriangleIndexCount(1, 2)).IsEqualTo(0);
        await Assert.That(ClothSheetPreset.CreateTriangleIndices(2, 1)).IsEmpty();
    }

    [Test]
    public async Task ClothSheetPreset_ApplyPins_SkipsInvalidIndex()
    {
        var spheres = new List<SphereState> { new(Vector3.Zero, Vector3.Zero) };
        ClothSheetPreset.ApplyPins(spheres, [5], [new Vector3(1, 2, 3)]);
        await Assert.That(spheres[0].Position).IsEqualTo(Vector3.Zero);
    }

    [Test]
    public async Task ClothSheetPreset_WriteTriangleIndices_ReturnsZeroWhenBufferTooSmall()
    {
        Span<int> buffer = stackalloc int[3];
        await Assert.That(ClothSheetPreset.WriteTriangleIndices(3, 2, buffer)).IsEqualTo(0);
    }

    [Test]
    public async Task ClothSheetPreset_BuildHanging_CornerPinModes()
    {
        var spheres = new List<SphereState>();
        var joints = new List<DistanceJoint>();
        var pins = new List<int>();
        var anchors = new List<Vector3>();
        var options = new ClothSheetOptions { Columns = 4, Rows = 3, PinMode = ClothPinMode.TopCorners };
        ClothSheetPreset.BuildHanging(Vector3.Zero, Vector3.UnitX, -Vector3.UnitY, options, spheres, joints, pins, anchors);
        await Assert.That(pins.Count).IsEqualTo(2);

        pins.Clear();
        anchors.Clear();
        options = new ClothSheetOptions { Columns = 4, Rows = 3, PinMode = ClothPinMode.Corners };
        ClothSheetPreset.BuildHanging(Vector3.Zero, Vector3.UnitX, -Vector3.UnitY, options, spheres, joints, pins, anchors);
        await Assert.That(pins.Count).IsEqualTo(4);
    }

    [Test]
    public async Task ClothSheetSimulator_EdgeCasesAndPinnedStretch()
    {
        var sim = new ClothSheetSimulator();
        _ = sim.Options;
        _ = sim.Joints;
        sim.SetJoints(Array.Empty<DistanceJoint>());
        sim.SetJoints(new List<DistanceJoint>());
        sim.ResetPileState();
        sim.MarkPileUnsettled();

        var spheres = new List<SphereState>
        {
            new(new Vector3(0, 2, 0), Vector3.Zero),
            new(new Vector3(0, 1.5f, 0), Vector3.Zero),
        };
        sim.SetJoints([new DistanceJoint(0, 1, restLength: 0.5f)]);
        sim.SetPins([0], [new Vector3(0, 2, 0)]);
        sim.MaxStretchRatio = 1f;
        sim.WindAcceleration = Vector3.Zero;

        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var interior = new InteriorClampVolume { MinX = -5, MaxX = 5, MinY = 0, MaxY = 5, MinZ = -5, MaxZ = 5 };

        sim.Step(world, spheres, interior, 0f);
        await Assert.That(sim.LastJointCorrections).IsEqualTo(0);

        spheres[1] = new SphereState(new Vector3(0, 0.2f, 0), new Vector3(0, -2, 0));
        sim.MaxStretchRatio = 1.05f;
        sim.Step(world, spheres, interior, 1f / 60f);
        await Assert.That(sim.LastJointCorrections).IsGreaterThanOrEqualTo(0);

        sim.SetJoints([new DistanceJoint(0, 99, restLength: 1f)]);
        sim.Step(world, spheres, interior, 1f / 60f);
        await Assert.That(sim.LastStats.ActiveCount).IsGreaterThanOrEqualTo(0);
    }

    private sealed class FlatRangeTerrain(float extent, float groundY) : IProjectileTerrainContact
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

    /// <summary>Range exit only via IsInside (TryTerrainHit / fallback paths), not segment tests.</summary>
    private sealed class NoSegmentExitTerrain(float extent, float groundY) : IProjectileTerrainContact
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

        public bool TrySegmentLeavesRange(Vector3 from, Vector3 to, out Vector3 hitPoint, out float fractionAlongSegment)
        {
            hitPoint = default;
            fractionAlongSegment = 1f;
            return false;
        }
    }

    /// <summary>Defers heightfield contact until TryTerrainHit / fallback (not the initial full-step check).</summary>
    private sealed class DelayedHeightfieldTerrain(float extent, float groundY, float radius) : IProjectileTerrainContact
    {
        private readonly AxisAlignedRangeBox _box = new(extent);
        private int _heightChecks;

        public bool IsInside(float x, float z) => _box.IsInside(x, z);

        public bool TryHeightfieldContact(Vector3 position, float checkRadius)
        {
            if (++_heightChecks <= 1)
                return false;
            return position.Y <= groundY + radius;
        }

        public Vector3 ProjectOntoSurface(Vector3 position, float surfaceEpsilon = 0.05f) =>
            new(
                System.Math.Clamp(position.X, 0f, extent),
                groundY + surfaceEpsilon,
                System.Math.Clamp(position.Z, 0f, extent));

        public bool TrySegmentLeavesRange(Vector3 from, Vector3 to, out Vector3 hitPoint, out float fractionAlongSegment) =>
            _box.TrySegmentLeavesRange(from, to, out hitPoint, out fractionAlongSegment);
    }
}
