using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;
using System.Numerics;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class UniformGridSphereContactSolverExtendedTests
{
    [Test]
    public async Task Resolve_WithImpulses_SeparatesAndChangesVelocity()
    {
        var spheres = new List<SphereState>
        {
            new(new Vector3(0f, 0f, 0f), new Vector3(1f, 0f, 0f)),
            new(new Vector3(0.15f, 0f, 0f), new Vector3(-1f, 0f, 0f)),
        };
        var soa = new SphereSoA();
        soa.SyncFrom(spheres);
        var solver = new UniformGridSphereContactSolver();
        var result = solver.Resolve(soa, radius: 0.22f, gridCellSize: 0.5f, restitution: 0.5f, applyImpulses: true, awakePairsOnly: false);
        soa.SyncTo(spheres);
        await Assert.That(result.Contacts).IsGreaterThan(0);
        await Assert.That(result.PairChecks).IsGreaterThan(0);
        await Assert.That(Vector3.Distance(spheres[0].Position, spheres[1].Position)).IsGreaterThan(0.4f);
    }

    [Test]
    public async Task Resolve_AwakePairsOnly_IgnoresSleepingPairs()
    {
        var spheres = new List<SphereState>
        {
            new(new Vector3(0f, 0f, 0f), Vector3.Zero) { IsSleeping = true },
            new(new Vector3(0.1f, 0f, 0f), Vector3.Zero) { IsSleeping = true },
        };
        var soa = new SphereSoA();
        soa.SyncFrom(spheres);
        var solver = new UniformGridSphereContactSolver();
        var result = solver.Resolve(soa, 0.22f, 0.5f, 0.8f, applyImpulses: true, awakePairsOnly: true);
        await Assert.That(result.Contacts).IsEqualTo(0);
    }
}

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class AngularLimitSolverExtendedTests
{
    [Test]
    public async Task SolveSwing_WithFrameReference_UsesBoneFrame()
    {
        var hip = new Vector3(0f, 0f, 0f);
        var chest = new Vector3(0f, 1f, 0f);
        var hand = new Vector3(0.6f, 1.2f, 0f);
        var spheres = new List<SphereState>
        {
            new(hip, Vector3.Zero),
            new(chest, Vector3.Zero),
            new(hand, Vector3.Zero),
        };
        BoneFrame.TryCreate(hip, chest, out var frame);
        var restLocal = frame.WorldToLocal(Vector3.Normalize(hand - hip));
        SwingLimit[] limits =
        [
            SwingLimit.CreateLocal(0, 2, frameReferenceSphere: 1, restLocal, maxRadians: 0.35f, stiffness: 1f),
        ];
        spheres[2] = new SphereState(new Vector3(-0.5f, 1.5f, -0.3f), Vector3.Zero);
        var corrections = AngularLimitSolver.Solve(limits, ReadOnlySpan<HingeLimit>.Empty, spheres, iterations: 12);
        await Assert.That(corrections).IsGreaterThan(0);
    }

    [Test]
    public async Task SolveHinge_InvalidAxis_ReturnsZero()
    {
        var spheres = new List<SphereState>
        {
            new(Vector3.Zero, Vector3.Zero),
            new(new Vector3(0f, 1f, 0f), Vector3.Zero),
        };
        HingeLimit[] limits =
        [
            new(0, 1, Vector3.Zero, Vector3.Zero, minRadians: 0f, maxRadians: 1f, stiffness: 1f),
        ];
        var corrections = AngularLimitSolver.Solve(ReadOnlySpan<SwingLimit>.Empty, limits, spheres, iterations: 4);
        await Assert.That(corrections).IsEqualTo(0);
    }
}
