using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;
using System.Numerics;
using TUnit.Core;

namespace Novolis.Physics.Unit;

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
