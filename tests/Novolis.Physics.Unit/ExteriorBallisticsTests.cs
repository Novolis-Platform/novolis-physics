using System.Numerics;
using Novolis.Physics.Ballistics;
using TUnit.Core;

namespace Novolis.Physics.Unit;

/// <summary>Speed of sound, the educational Mach curve, and Coriolis direction. Constant-Cd vacuum range stays in the older tests.</summary>
public sealed class ExteriorBallisticsTests
{
    [Test]
    public async Task SpeedOfSound_AtStandardTemperature_IsAbout340()
    {
        var speed = StandardAtmosphere.SpeedOfSound(288.15);
        await Assert.That(speed).IsGreaterThan(340.2).And.IsLessThan(340.5);
    }

    [Test]
    public async Task BoatTail_InterpolatesTheTransonicRise()
    {
        var law = DragCoefficientLaw.BoatTail;
        await Assert.That(law.Evaluate(0.0)).IsEqualTo(0.17);
        await Assert.That(law.Evaluate(1.15)).IsEqualTo(0.41);
        await Assert.That(law.Evaluate(0.9)).IsEqualTo(0.255).Within(1e-9);
        await Assert.That(law.Evaluate(4.0)).IsEqualTo(0.25);
    }

    [Test]
    public async Task Coriolis_NorthwardShotAtSixtyNorth_AcceleratesEast()
    {
        var accel = CoriolisAcceleration.EastUpNorth(60.0, new Vector3(0f, 0f, 100f));
        await Assert.That(accel.X).IsGreaterThan(0.005f);
        await Assert.That(accel.Z).IsEqualTo(0f).Within(1e-6f);
    }

    [Test]
    public async Task Coriolis_EastwardShotAtSixtyNorth_AcceleratesSouthAndUp()
    {
        var accel = CoriolisAcceleration.EastUpNorth(60.0, new Vector3(100f, 0f, 0f));
        await Assert.That(accel.Z).IsLessThan(-0.005f);
        await Assert.That(accel.Y).IsGreaterThan(0.005f);
    }

    [Test]
    public async Task Coriolis_NorthwardShotAtSixtySouth_AcceleratesWest()
    {
        var accel = CoriolisAcceleration.EastUpNorth(-60.0, new Vector3(0f, 0f, 100f));
        await Assert.That(accel.X).IsLessThan(-0.005f);
    }

    [Test]
    public async Task IntegratedNorthwardVacuumShot_DriftsEast()
    {
        var simulation = new ProjectileBallisticSimulation();
        var environment = new ProjectileBallisticEnvironment(9.80665, 0.0, default, 288.15, 60.0);
        var state = new ProjectileState(new Vector3(0f, 8_000f, 0f), new Vector3(0f, 0f, 600f), massKg: 47.0, timeSeconds: 0);
        const double dt = 1.0 / 120.0;
        for (var i = 0; i < 15 * 120; i++)
            state = simulation.Step(state, dt, environment);

        await Assert.That(state.Position.X).IsGreaterThan(5f);
    }

    [Test]
    public async Task BoatTailLaw_KeepsMoreSpeedThanConstantCd_AboveMachTwo()
    {
        var area = global::System.Math.PI * 0.155 * 0.155 / 4.0;
        var profile = new ProjectileProfile(47.0, area, dragCoefficient: 0.42);
        var constant = new ProjectileBallisticSimulation(profile);
        var mach = new ProjectileBallisticSimulation(profile, DragCoefficientLaw.BoatTail);
        var environment = new ProjectileBallisticEnvironment(9.80665, 1.2, default, 288.15);
        var start = new ProjectileState(Vector3.Zero, new Vector3(700f, 200f, 0f), massKg: 47.0, timeSeconds: 0);
        var plain = start;
        var curved = start;
        const double dt = 1.0 / 120.0;
        for (var i = 0; i < 2 * 120; i++)
        {
            plain = constant.Step(plain, dt, environment);
            curved = mach.Step(curved, dt, environment);
        }

        await Assert.That(curved.Velocity.Length()).IsGreaterThan(plain.Velocity.Length());
    }
}
