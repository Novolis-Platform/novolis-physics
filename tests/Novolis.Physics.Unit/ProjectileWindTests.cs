using Novolis.Physics.Ballistics;
using System.Numerics;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class ProjectileWindTests
{
    [Test]
    public async Task Crosswind_DeflectsTrajectory_Downrange()
    {
        var profile = new ProjectileProfile(10, 0.01, 0.47);
        var sim = new ProjectileBallisticSimulation(profile);
        var noWind = new ProjectileBallisticEnvironment(9.80665, 1.225);
        var withWind = new ProjectileBallisticEnvironment(9.80665, 1.225, new Vector3(0f, 0f, 12f));

        var state = new ProjectileState(new Vector3(0, 0, 0), new Vector3(120, 35, 0), massKg: 10);
        const double dt = 1.0 / 120.0;

        for (var i = 0; i < 2400; i++)
        {
            state = sim.Step(state, dt, noWind);
        }

        var windState = new ProjectileState(new Vector3(0, 0, 0), new Vector3(120, 35, 0), massKg: 10);
        for (var i = 0; i < 2400; i++)
        {
            windState = sim.Step(windState, dt, withWind);
        }

        await Assert.That(windState.Position.Z).IsGreaterThan(state.Position.Z + 5f);
    }
}
