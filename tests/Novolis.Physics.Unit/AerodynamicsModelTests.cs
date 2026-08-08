using Novolis.Physics.TestSupport;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Aerodynamics;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class AerodynamicsModelTests
{
    [Test]
    public async Task ExponentialAtmosphere_MatchesExpLaw()
    {
        var atm = new ExponentialAtmosphereModel(seaLevelDensityKgPerM3: 1.225, scaleHeightMeters: 8500.0);
        var h = 17_000.0;
        var expected = 1.225 * global::System.Math.Exp(-h / 8500.0);
        var rho = atm.DensityAtAltitude(h);

        var o = NovolisPhysicsTestTrace.Out;
        PhysicsDashboard.SectionAndTable(
            o,
            "Exponential atmosphere",
            new[] { new DensityRow("sea level", 0, atm.DensityAtAltitude(0)), new DensityRow("17 km", h, rho) },
            new TableOptions { MaxCellWidth = 20, RightAlignNumericColumns = true },
            tableCaption: "rho(h) = rho0 * exp(-h/H)");

        await Assert.That(global::System.Math.Abs(rho - expected)).IsLessThanOrEqualTo((float)(1e-12));
        await Assert.That(atm.DensityAtAltitude(0)).IsEqualTo(1.225);
    }

    [Test]
    public async Task SimpleLiftDrag_DragOpposesRelativeVelocity_WithWind()
    {
        var atm = new ExponentialAtmosphereModel(1.225, 8500);
        var wind = PhysicsTestVectors.V(5, 0, 0);
        var bodyVel = PhysicsTestVectors.V(25, 0, 0);
        var body = new RigidBodyState(
            Vector3.Zero,
            bodyVel,
            Quaternion.Identity,
            Vector3.Zero,
            mass: 10,
            inertiaDiagonalBody: PhysicsTestVectors.V(1, 1, 1));
        var forward = PhysicsTestVectors.V(0, 0, 1);
        var env = new SimpleAeroEnvironment(atm, altitudeMeters: 0, wind, referenceAreaM2: 2, dragCoefficient: 0.4, liftCoefficient: 0, liftReferenceForwardWorld: forward);
        var model = new SimpleLiftDragModel();
        var f = model.Evaluate(body, env, 0).Force;
        var vRel = bodyVel - wind;

        var o = NovolisPhysicsTestTrace.Out;
        o.Results("Lift/drag - relative wind outcome");
        o.Table(
            new[]
            {
                new AeroSampleRow("v_rel.X", vRel.X),
                new AeroSampleRow("F.X", f.X),
                new AeroSampleRow("F.Y", f.Y),
                new AeroSampleRow("F.Z", f.Z),
            },
            new TableOptions { MaxCellWidth = 18, RightAlignNumericColumns = true },
            caption: "expect drag roughly opposite v_rel (lift adds orthogonal component)");

        var dot = Vector3.Dot(f, vRel);
        await Assert.That(dot).IsLessThan((float)(0));
    }

    [Test]
    public async Task SimpleLiftDrag_ZeroRelativeVelocity_ReturnsZero()
    {
        var atm = new ExponentialAtmosphereModel(1.225, 8500);
        var wind = PhysicsTestVectors.V(10, 0, 0);
        var body = new RigidBodyState(
            Vector3.Zero,
            PhysicsTestVectors.V(10, 0, 0),
            Quaternion.Identity,
            Vector3.Zero,
            1,
            PhysicsTestVectors.V(1, 1, 1));
        var env = new SimpleAeroEnvironment(atm, 0, wind, 1, 0.5, 0.2, PhysicsTestVectors.V(0, 1, 0));
        var f = new SimpleLiftDragModel().Evaluate(body, env, 0).Force;
        await Assert.That(f.Length()).IsLessThanOrEqualTo((float)(1e-8));
    }

    private sealed record DensityRow(string Label, double AltitudeM, double DensityKgM3);

    private sealed record AeroSampleRow(string Label, double Value);
}
