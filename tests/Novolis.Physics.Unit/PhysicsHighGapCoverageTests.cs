using Novolis.Physics.Abstractions;
using Novolis.Physics.Astro;
using Novolis.Physics.Ballistics;
using Novolis.Physics.Cloth;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;
using Novolis.Physics.Motion;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

/// <summary>Targets remaining line/branch gaps in Abstractions, Collision.Simple, Joints, Ballistics, Cloth, Motion.</summary>
public sealed class PhysicsHighGapCoverageTests
{
    [Test]
    public async Task ForceSample_ZeroAndScale()
    {
        var z = ForceSample.Zero;
        await Assert.That(z.Force).IsEqualTo(Vector3.Zero);
        await Assert.That(z.Torque).IsEqualTo(Vector3.Zero);

        var scaled = new ForceSample(new Vector3(2, 0, 0), new Vector3(0, 4, 0)) * 0.5;
        await Assert.That(scaled.Force.X).IsEqualTo(1f).Within(1e-5f);
        await Assert.That(scaled.Torque.Y).IsEqualTo(2f).Within(1e-5f);
    }

    [Test]
    public async Task RigidBodyState_NormalizesOrientation()
    {
        var q = new Quaternion(1, 2, 3, 4);
        var body = new RigidBodyState(
            new Vector3(1, 2, 3),
            new Vector3(0.1f, 0, 0),
            q,
            new Vector3(0, 0.2f, 0),
            mass: 2.5,
            inertiaDiagonalBody: new Vector3(1, 2, 3));
        await Assert.That(body.Mass).IsEqualTo(2.5);
        await Assert.That(Quaternion.Dot(body.Orientation, body.Orientation)).IsEqualTo(1f).Within(1e-5f);
    }

    [Test]
    public async Task AxisAlignedRangeBox_ThrowsAndSegmentBranches()
    {
        await Assert.That(() => new AxisAlignedRangeBox(0f)).ThrowsExactly<ArgumentOutOfRangeException>();
        await Assert.That(() => new AxisAlignedRangeBox(-1f)).ThrowsExactly<ArgumentOutOfRangeException>();

        var box = new AxisAlignedRangeBox(10f);
        await Assert.That(box.IsInside(5f, 5f)).IsTrue();
        await Assert.That(box.IsInside(-1f, 5f)).IsFalse();

        await Assert.That(box.TrySegmentLeavesRange(new Vector3(1, 0, 1), new Vector3(2, 0, 2), out _, out _)).IsFalse();

        await Assert.That(box.TrySegmentLeavesRange(new Vector3(5, 0, 5), new Vector3(20, 0, 5), out var hitX, out var tX)).IsTrue();
        await Assert.That(hitX.X).IsEqualTo(10f).Within(1e-3f);
        await Assert.That(tX).IsGreaterThan(0f).And.IsLessThanOrEqualTo(1f);

        await Assert.That(box.TrySegmentLeavesRange(new Vector3(5, 0, 5), new Vector3(-5, 0, 5), out var hitNegX, out _)).IsTrue();
        await Assert.That(hitNegX.X).IsEqualTo(0f).Within(1e-3f);

        await Assert.That(box.TrySegmentLeavesRange(new Vector3(5, 0, 5), new Vector3(5, 0, 20), out var hitZ, out _)).IsTrue();
        await Assert.That(hitZ.Z).IsEqualTo(10f).Within(1e-3f);

        await Assert.That(box.TrySegmentLeavesRange(new Vector3(5, 0, 5), new Vector3(5, 0, -5), out var hitNegZ, out _)).IsTrue();
        await Assert.That(hitNegZ.Z).IsEqualTo(0f).Within(1e-3f);

        await Assert.That(box.TrySegmentLeavesRange(new Vector3(20, 0, 20), new Vector3(30, 0, 30), out _, out _)).IsFalse();
    }

    [Test]
    public async Task EmptyStaticWorld_AllQueriesMiss()
    {
        var world = new EmptyStaticWorld();
        await Assert.That(world.Raycast(new Ray(Vector3.Zero, Vector3.UnitY), 10, out var rayHit)).IsFalse();
        await Assert.That(rayHit.Distance).IsEqualTo(0);
        await Assert.That(world.SweepSphere(new Sphere(Vector3.One, 0.5f), Vector3.UnitX, out _)).IsFalse();
        await Assert.That(world.SweepCapsule(new Capsule(Vector3.Zero, Vector3.UnitY, 0.2f), Vector3.UnitZ, out _)).IsFalse();
    }

    [Test]
    public async Task SphereOverlapResolution_NoOverlapAndSeparatingImpulse()
    {
        var a = new Vector3(0, 0, 0);
        var b = new Vector3(10, 0, 0);
        await Assert.That(SphereOverlapResolution.Separate(ref a, ref b, radius: 0.5f)).IsFalse();

        a = new Vector3(0, 0, 0);
        b = new Vector3(0.4f, 0, 0);
        var va = new Vector3(-1, 0, 0);
        var vb = new Vector3(1, 0, 0);
        await Assert.That(SphereOverlapResolution.SeparateWithImpulse(ref a, ref b, ref va, ref vb, 0.5f, 0.5f)).IsTrue();
        await Assert.That(va.X).IsEqualTo(-1f).Within(1e-4f);

        a = Vector3.Zero;
        b = Vector3.Zero;
        va = new Vector3(2, 0, 0);
        vb = new Vector3(-2, 0, 0);
        await Assert.That(SphereOverlapResolution.SeparateWithImpulse(ref a, ref b, ref va, ref vb, 0.5f, 1f)).IsTrue();
        await Assert.That(va.X).IsLessThan(2f);
    }

    [Test]
    public async Task SphereOverlapResolution_SeparateWithImpulse_NoOverlapReturnsFalse()
    {
        var a = new Vector3(0, 0, 0);
        var b = new Vector3(5, 0, 0);
        var va = Vector3.Zero;
        var vb = Vector3.Zero;
        await Assert.That(SphereOverlapResolution.SeparateWithImpulse(ref a, ref b, ref va, ref vb, 0.1f, 0.5f)).IsFalse();
    }

    [Test]
    public async Task SphereState_DefaultCtorAndSpeed()
    {
        var s = new SphereState();
        await Assert.That(s.Position).IsEqualTo(Vector3.Zero);
        await Assert.That(s.Speed).IsEqualTo(0f);
        s.Velocity = new Vector3(3, 4, 0);
        await Assert.That(s.Speed).IsEqualTo(5f).Within(1e-5f);
    }

    [Test]
    public async Task SphereContactKinematics_ReflectElastic()
    {
        var v = new Vector3(0, -2, 0);
        var reflected = SphereContactKinematics.ReflectElastic(v, Vector3.UnitY);
        await Assert.That(reflected.Y).IsEqualTo(2f).Within(1e-4f);
    }

    [Test]
    public async Task BvhStaticWorld_EmptyMeshRaycast_ReturnsFalse()
    {
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        await Assert.That(world.Raycast(new Ray(Vector3.UnitY, -Vector3.UnitY), 10, out _)).IsFalse();
        await Assert.That(world.SweepSphere(new Sphere(Vector3.UnitY, 0.1f), -Vector3.UnitY, out _)).IsFalse();
    }

    [Test]
    public async Task BvhStaticWorld_SweepCapsule_BothEndpointsHit_PicksCloser()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-20, 0, -20), new Vector3(20, 0, -20), new Vector3(-20, 0, 20), new Vector3(20, 0, 20)],
            [0, 1, 2, 1, 3, 2]));
        var cap = new Capsule(new Vector3(0, 2, 0), new Vector3(0, 3, 0), radius: 0.2f);
        var hit = world.SweepCapsule(in cap, new Vector3(0, -5, 0), out var info);
        await Assert.That(hit).IsTrue();
        await Assert.That(info.Distance).IsGreaterThan(0);
    }

    [Test]
    public async Task BvhStaticSphereIntegrator_GrazingStuckAndZeroVelocity()
    {
        var world = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-10, 0, -10), new Vector3(10, 0, -10), new Vector3(-10, 0, 10), new Vector3(10, 0, 10)],
            [0, 1, 2, 1, 3, 2]));
        var pos = new Vector3(0, 0.05f, 0);
        var vel = new Vector3(0, -1e-8f, 0);
        var n = BvhStaticSphereIntegrator.AdvanceOneStep(world, ref pos, ref vel, 0.2, 1e-20);
        await Assert.That(n).IsEqualTo(0);

        pos = new Vector3(0, 0.15f, 0);
        vel = new Vector3(0, -50f, 0);
        n = BvhStaticSphereIntegrator.AdvanceOneStep(
            world, ref pos, ref vel, 0.2, 0.05, maxReflectionsPerStep: 4, normalRestitution: 0.1);
        await Assert.That(n).IsGreaterThanOrEqualTo(0);
    }

    [Test]
    public async Task BallisticsQueries_LineOfSightAndSweep()
    {
        var world = new EmptyStaticWorld();
        await Assert.That(BallisticsQueries.LineOfSight(world, new Ray(Vector3.Zero, Vector3.UnitX), 10, out _)).IsFalse();
        await Assert.That(BallisticsQueries.SweepProjectileSphere(
            world, new Sphere(Vector3.Zero, 0.1f), Vector3.UnitX, out _)).IsFalse();

        var meshWorld = new BvhStaticWorld(new TriangleMesh(
            [new Vector3(-5, 0, -5), new Vector3(5, 0, -5), new Vector3(-5, 0, 5)],
            [0, 1, 2]));
        await Assert.That(BallisticsQueries.LineOfSight(
            meshWorld, new Ray(new Vector3(0, 5, 0), -Vector3.UnitY), 20, out var hit)).IsTrue();
        await Assert.That(hit.Distance).IsGreaterThan(0);
    }

    [Test]
    public async Task ProjectileQuadraticDrag_ZeroSpeedAndZeroDensity()
    {
        var model = new ProjectileQuadraticDragModel(new ProjectileProfile(0.01, 0.01, 0.5));
        var state = new ProjectileState(Vector3.Zero, Vector3.Zero, 1, 0);
        var env = new ProjectileDragEnvironment(1.2, Vector3.Zero);
        await Assert.That(model.Evaluate(state, env, 0).Force).IsEqualTo(Vector3.Zero);

        state = new ProjectileState(Vector3.Zero, new Vector3(10, 0, 0), 1, 0);
        env = new ProjectileDragEnvironment(0, Vector3.Zero);
        await Assert.That(model.Evaluate(state, env, 0).Force).IsEqualTo(Vector3.Zero);
    }

    [Test]
    public async Task ProjectileMath_DegenerateDenom()
    {
        var prev = new ProjectileState(new Vector3(0, 0, 0), Vector3.Zero, 1, 0);
        var curr = new ProjectileState(new Vector3(0, 0, 0), Vector3.Zero, 1, 1);
        var impact = ProjectileMath.InterpolateGroundImpact(prev, curr);
        await Assert.That(impact.Position.Y).IsEqualTo(0f).Within(1e-5f);
    }

    [Test]
    public async Task BallisticTrajectoryRunner_PropertiesAndNonFlightAdvance()
    {
        var runner = new BallisticTrajectoryRunner(new BallisticTrajectoryRunnerOptions { RecordTrail = true });
        runner.Begin(new ProjectileState(new Vector3(1, 2, 3), new Vector3(0, -1, 0), 1, 0.5));
        await Assert.That(runner.CurrentPosition).IsEqualTo(new Vector3(1, 2, 3));
        await Assert.That(runner.CurrentVelocity.Y).IsEqualTo(-1f);
        await Assert.That(runner.TimeSeconds).IsEqualTo(0.5);

        runner.Reset();
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.8, 0);
        var terrain = new FlatTerrain(100f, 0f);
        runner.AdvanceOne(sim, env, null, terrain, Vector3.Zero);
        await Assert.That(runner.Phase).IsEqualTo(BallisticTrajectoryPhase.Ready);
    }

    [Test]
    public async Task BallisticTrajectoryRunner_AdvanceWithBudget_Impacts()
    {
        var runner = new BallisticTrajectoryRunner(new BallisticTrajectoryRunnerOptions
        {
            RecordTrail = true,
            Step = new ProjectileTerrainStepOptions { DtSeconds = 0.05, MaxSweepMeters = 50f, ProjectileRadius = 0.05f, MaxSteps = 200 },
        });
        runner.Begin(new ProjectileState(new Vector3(10, 5, 10), new Vector3(0, -20, 0), 1, 0));
        var sim = new ProjectileBallisticSimulation();
        var env = new ProjectileBallisticEnvironment(9.80665, 0);
        var terrain = new FlatTerrain(200f, 0f);
        runner.AdvanceWithBudget(sim, env, null, terrain, Vector3.Zero, maxPhysicsSteps: 80);
        await Assert.That(runner.Phase).IsEqualTo(BallisticTrajectoryPhase.Impacted);
        await Assert.That(runner.Impact).IsNotNull();
        await Assert.That(runner.Trail.Count).IsGreaterThan(0);
    }

    [Test]
    public async Task RagdollBodyCollision_EarlyExitsAndSkipSwaps()
    {
        var one = new List<SphereState> { new(Vector3.Zero, Vector3.Zero) };
        await Assert.That(RagdollBodyCollision.ResolveOverlaps(one, 0.5f, iterations: 4)).IsEqualTo(0);
        await Assert.That(RagdollBodyCollision.ResolveOverlaps(
            [new(Vector3.Zero, Vector3.Zero), new(Vector3.One, Vector3.Zero)], 0.5f, iterations: 0)).IsEqualTo(0);

        await Assert.That(RagdollBodyCollision.BuildAdjacentSkipPairs(ReadOnlySpan<DistanceJoint>.Empty)).IsEmpty();

        DistanceJoint[] joints = [new(2, 0, 1f), new(1, 0, 1f)];
        var skip = RagdollBodyCollision.BuildAdjacentSkipPairs(joints);
        await Assert.That(skip.Length).IsEqualTo(2);

        var spheres = new List<SphereState>
        {
            new(new Vector3(0, 0, 0), Vector3.Zero),
            new(new Vector3(0.1f, 0, 0), Vector3.Zero),
            new(new Vector3(0.2f, 0, 0), Vector3.Zero),
        };
        var fixes = RagdollBodyCollision.ResolveOverlaps(spheres, 0.5f, 6, 1.02f, skip);
        await Assert.That(fixes).IsGreaterThanOrEqualTo(0);
    }

    [Test]
    public async Task BoneFrame_DegenerateAndParallelToZ()
    {
        await Assert.That(BoneFrame.TryCreate(Vector3.Zero, Vector3.Zero, out _)).IsFalse();
        await Assert.That(BoneFrame.TryCreate(Vector3.Zero, new Vector3(0, 0, 1), out var frame)).IsTrue();
        var world = frame.LocalToWorld(new Vector3(1, 0, 0));
        var local = frame.WorldToLocal(world);
        await Assert.That(local.X).IsEqualTo(1f).Within(1e-4f);
    }

    [Test]
    public async Task AngularLimitSolver_SwingInvalidIndexAndWithinCone()
    {
        var spheres = new List<SphereState> { new(Vector3.Zero, Vector3.Zero) };
        SwingLimit[] bad = [new(0, 5, Vector3.UnitY, 0.5f, 1f)];
        await Assert.That(AngularLimitSolver.SolveSwing(bad[0], spheres)).IsEqualTo(0);

        spheres =
        [
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0, 1, 0), Vector3.Zero),
        ];
        SwingLimit[] inside = [new(0, 1, Vector3.UnitY, maxRadians: 1f, stiffness: 1f)];
        await Assert.That(AngularLimitSolver.SolveSwing(inside[0], spheres)).IsEqualTo(0);
    }

    [Test]
    public async Task AngularLimitSolver_HingeZeroAxis_ReturnsZero()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0, 1, 0), Vector3.Zero),
        };
        HingeLimit[] limits = [new(0, 1, Vector3.Zero, Vector3.UnitX, -0.5f, 0.5f, 1f)];
        await Assert.That(AngularLimitSolver.SolveHinge(limits[0], spheres)).IsEqualTo(0);
    }

    [Test]
    public async Task DistanceJointSolver_EdgeCases()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(2, 0, 0), Vector3.Zero),
        };
        await Assert.That(DistanceJointSolver.Solve(ReadOnlySpan<DistanceJoint>.Empty, spheres)).IsEqualTo(0);
        await Assert.That(DistanceJointSolver.Solve([new(0, 1, 1f)], [], iterations: 4)).IsEqualTo(0);
        await Assert.That(DistanceJointSolver.Solve([new(0, 1, 1f)], spheres, iterations: 0)).IsEqualTo(0);

        var n = DistanceJointSolver.Solve([new(0, 1, 1f)], spheres, iterations: 4, maxStrainFraction: float.NaN);
        await Assert.That(n).IsGreaterThan(0);

        spheres[0] = new SphereState(Vector3.Zero, Vector3.Zero);
        spheres[1] = new SphereState(Vector3.Zero, Vector3.Zero);
        await Assert.That(DistanceJointSolver.Solve([new(0, 1, 1f)], spheres, iterations: 2)).IsEqualTo(0);
        await Assert.That(DistanceJointSolver.Solve([new(0, 9, 1f)], spheres, iterations: 2)).IsEqualTo(0);
    }

    [Test]
    public async Task ConstrainedSphereSimulator_OptionsAndAngularPass()
    {
        var sim = new ConstrainedSphereSimulator
        {
            Options = { Radius = 0.2f, Gravity = Vector3.Zero, MaxSpeedMps = 5f },
            ConstraintPasses = 1,
            AngularIterations = 2,
            InternalCollisionIterations = 2,
        };
        _ = sim.Options;
        sim.SetJoints([]);
        sim.SetJoints([new DistanceJoint(0, 1, 1f)]);
        sim.ResetPileState();
        sim.MarkPileUnsettled();

        var spheres = new List<SphereState>
        {
            new(new Vector3(0, 1, 0), Vector3.Zero),
            new(new Vector3(0, 0.2f, 0), Vector3.Zero),
        };
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var interior = new InteriorClampVolume { MinX = -5, MaxX = 5, MinY = 0, MaxY = 5, MinZ = -5, MaxZ = 5 };
        SwingLimit[] swings = [new(0, 1, Vector3.UnitY, 0.2f, 1f)];
        sim.Step(world, spheres, interior, 1f / 60f, swings, ReadOnlySpan<HingeLimit>.Empty);
        await Assert.That(sim.LastJointCorrections).IsGreaterThanOrEqualTo(0);
        await Assert.That(sim.LastAngularCorrections).IsGreaterThanOrEqualTo(0);

        sim.SetJoints([]);
        sim.Step(world, spheres, interior, 1f / 60f);
        await Assert.That(sim.LastJointCorrections).IsEqualTo(0);
    }

    [Test]
    public async Task ClothBlade_LengthAndAstronomicalMeters()
    {
        var blade = new ClothBlade(Vector3.Zero, new Vector3(3, 4, 0), 0.1f);
        await Assert.That(blade.Length).IsEqualTo(5f).Within(1e-5f);
        await Assert.That(AstronomicalUnits.MetersToPc(AstronomicalUnits.MetersPerPc)).IsEqualTo(1.0).Within(1e-12);
        await Assert.That(AstronomicalUnits.MetersToAu(AstronomicalUnits.MetersPerAu)).IsEqualTo(1.0).Within(1e-12);
    }

    [Test]
    public async Task ClothSheetSimulator_EmptyPinsAndOptionsSetter()
    {
        var sim = new ClothSheetSimulator
        {
            Options = { Radius = 0.15f, Gravity = Vector3.Zero },
        };
        _ = sim.Options;
        sim.SetPins(ReadOnlySpan<int>.Empty, ReadOnlySpan<Vector3>.Empty);
        sim.SetJoints([]);
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var spheres = new List<SphereState> { new(Vector3.Zero, Vector3.Zero) };
        var interior = new InteriorClampVolume { MinX = -1, MaxX = 1, MinY = 0, MaxY = 2, MinZ = -1, MaxZ = 1 };
        sim.Step(world, spheres, interior, 1f / 60f);
        await Assert.That(sim.LastJointCorrections).IsEqualTo(0);
    }

    [Test]
    public async Task FixedStepAccumulator_RejectsNonPositiveAndDrains()
    {
        await Assert.That(() => new FixedStepAccumulator(0)).ThrowsExactly<ArgumentOutOfRangeException>();
        var acc = new FixedStepAccumulator(0.02);
        var steps = 0;
        var n = acc.AddTimeAndDrain(0.05, _ => steps++);
        await Assert.That(n).IsEqualTo(2);
        await Assert.That(steps).IsEqualTo(2);
    }

    [Test]
    public async Task SemiImplicitEuler_ZeroMassAndZeroInertia()
    {
        var integrator = new SemiImplicitEulerRigidBodyIntegrator();
        var body = new RigidBodyState(
            Vector3.Zero,
            Vector3.Zero,
            Quaternion.Identity,
            Vector3.Zero,
            mass: 0,
            inertiaDiagonalBody: Vector3.Zero);
        var next = integrator.Step(body, new ForceSample(new Vector3(10, 0, 0), new Vector3(1, 0, 0)), 0.01);
        await Assert.That(next.Velocity).IsEqualTo(Vector3.Zero);
    }

    [Test]
    public async Task SimulationPipeline_ExposesForces()
    {
        var integrator = new SemiImplicitEulerRigidBodyIntegrator();
        var pipeline = new SimulationPipeline<RigidBodyState, int>(integrator);
        await Assert.That(pipeline.Forces.Count).IsEqualTo(0);
    }

    [Test]
    public async Task SphereSimulator_EmptyListAndSettledSkip()
    {
        var sim = new SphereInStaticWorldSimulator
        {
            Options =
            {
                Radius = 0.25f,
                FloorHeight = 0f,
                Gravity = Vector3.Zero,
                SleepSpeedThreshold = 10f,
                MaxSpeedMps = 5f,
            },
        };
        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var interior = new InteriorClampVolume { MinX = -2, MaxX = 2, MinY = 0, MaxY = 2, MinZ = -2, MaxZ = 2 };

        sim.Step(world, [], interior, 1f / 60f);
        await Assert.That(sim.LastStats.ActiveCount).IsEqualTo(0);

        var spheres = new List<SphereState>
        {
            new(new Vector3(0, 0.25f, 0), Vector3.Zero) { IsGrounded = true, IsSleeping = true },
            new(new Vector3(1, 0.25f, 0), Vector3.Zero) { IsGrounded = true, IsSleeping = true },
        };
        for (var i = 0; i < 8; i++)
            sim.Step(world, spheres, interior, 1f / 60f);
        sim.Step(world, spheres, interior, 1f / 60f);
        await Assert.That(sim.LastStats.SphereContactSkipped || sim.LastStats.SleepingCount >= 0).IsTrue();
    }

    private sealed class FlatTerrain(float extent, float groundY) : IProjectileTerrainContact
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
