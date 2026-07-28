using Novolis.Physics.Collision.Simple;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class UniformGridSphereContactSolverTests
{
    [Test]
    public async Task Resolve_SeparatesOverlappingSpheres()
    {
        var spheres = new List<SphereState>
        {
            new(new(0f, 0f, 0f), default),
            new(new(0.1f, 0f, 0f), default),
        };
        var soa = new SphereSoA();
        soa.SyncFrom(spheres);

        var solver = new UniformGridSphereContactSolver();
        var result = solver.Resolve(soa, radius: 0.22f, gridCellSize: 0.5f, restitution: 0.88f, applyImpulses: false, awakePairsOnly: false);

        soa.SyncTo(spheres);
        var delta = spheres[1].Position - spheres[0].Position;
        var dist = delta.Length();

        await Assert.That(result.Contacts).IsGreaterThan(0);
        await Assert.That(dist).IsGreaterThan(0.44f);
    }
}
