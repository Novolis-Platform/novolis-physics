using Novolis.Physics.TestSupport;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Gravity;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class PatchedConicGravityTests
{
    [Test]
    public async Task PatchedConic_InsidePrimarySoi_UsesPrimaryGmAndDirection()
    {
        const double mass = 50.0;
        var primaryGm = 4.0e12;
        const double soi = 8_000.0;
        var secondaryGm = 1.0e8;
        var field = new PatchedConicPairField(
            primaryPosition: Vector3.Zero,
            primaryGm,
            primarySphereOfInfluenceRadius: soi,
            secondaryPosition: PhysicsTestVectors.V(5_000_000.0, 0, 0),
            secondaryGm);

        var bodyPos = PhysicsTestVectors.V(2_000.0, 0, 0);
        var body = new RigidBodyState(bodyPos, Vector3.Zero, Quaternion.Identity, Vector3.Zero, mass, PhysicsTestVectors.V(1, 1, 1));
        var model = new PatchedConicGravityModel();
        var sample = model.Evaluate(body, field, timeSeconds: 0);

        var expected = NewtonTowardSource(bodyPos, Vector3.Zero, primaryGm, mass);
        var o = NovolisPhysicsTestTrace.Out;
        o.Section("Patched conic - inside primary SOI");
        o.Table(
            new[]
            {
                new ForceCompareRow("Fx", expected.X, sample.Force.X, global::System.Math.Abs(expected.X - sample.Force.X)),
                new ForceCompareRow("Fy", expected.Y, sample.Force.Y, global::System.Math.Abs(expected.Y - sample.Force.Y)),
                new ForceCompareRow("Fz", expected.Z, sample.Force.Z, global::System.Math.Abs(expected.Z - sample.Force.Z)),
            },
            new TableOptions { MaxCellWidth = 28 },
            caption: "Force vs hand inverse-square toward primary",
            columnPropertyOrder: new[] { "Label", "Expected", "Actual", "AbsError" });

        var rel = 1e-9 * global::System.Math.Max(1.0, expected.Length());
        await Assert.That((sample.Force - expected).Length()).IsLessThanOrEqualTo((float)(rel));
    }

    [Test]
    public async Task PatchedConic_OutsidePrimarySoi_UsesSecondarySource()
    {
        const double mass = 10.0;
        var primaryGm = 4.0e12;
        const double soi = 5_000.0;
        var secondaryGm = 9.0e11;
        var secondaryPos = PhysicsTestVectors.V(80_000.0, 0, 0);
        var field = new PatchedConicPairField(
            Vector3.Zero,
            primaryGm,
            soi,
            secondaryPos,
            secondaryGm);

        var bodyPos = PhysicsTestVectors.V(12_000.0, 0, 0);
        var body = new RigidBodyState(bodyPos, Vector3.Zero, Quaternion.Identity, Vector3.Zero, mass, PhysicsTestVectors.V(1, 1, 1));
        var model = new PatchedConicGravityModel();
        var sample = model.Evaluate(body, field, 0);

        var expected = NewtonTowardSource(bodyPos, secondaryPos, secondaryGm, mass);
        var o = NovolisPhysicsTestTrace.Out;
        o.Results("Patched conic - outside SOI outcome");
        o.Table(
            new[]
            {
                new ForceCompareRow("Fx", expected.X, sample.Force.X, global::System.Math.Abs(expected.X - sample.Force.X)),
                new ForceCompareRow("Fy", expected.Y, sample.Force.Y, global::System.Math.Abs(expected.Y - sample.Force.Y)),
                new ForceCompareRow("Fz", expected.Z, sample.Force.Z, global::System.Math.Abs(expected.Z - sample.Force.Z)),
            },
            new TableOptions { MaxCellWidth = 28 },
            caption: "12 km from primary (> SOI): force toward secondary at 80 km",
            columnPropertyOrder: new[] { "Label", "Expected", "Actual", "AbsError" });

        var rel = 1e-9 * global::System.Math.Max(1.0, expected.Length());
        await Assert.That((sample.Force - expected).Length()).IsLessThanOrEqualTo((float)(rel));
    }

    [Test]
    public async Task PatchedConic_OnSoiBoundary_InsideUsesPrimary()
    {
        const double mass = 1.0;
        var primaryGm = 1.0e6;
        const double soi = 10_000.0;
        var field = new PatchedConicPairField(
            Vector3.Zero,
            primaryGm,
            soi,
            PhysicsTestVectors.V(1e9, 0, 0),
            1.0e12);

        var onBoundary = PhysicsTestVectors.V(soi, 0, 0);
        var body = new RigidBodyState(onBoundary, Vector3.Zero, Quaternion.Identity, Vector3.Zero, mass, PhysicsTestVectors.V(1, 1, 1));
        var f = new PatchedConicGravityModel().Evaluate(body, field, 0).Force;
        var expected = NewtonTowardSource(onBoundary, Vector3.Zero, primaryGm, mass);
        await Assert.That((f - expected).Length()).IsLessThanOrEqualTo((float)(1e-9 * global::System.Math.Max(1.0, expected.Length())));
    }

    private static Vector3 NewtonTowardSource(Vector3 body, Vector3 source, double gm, double mass)
    {
        var r = source - body;
        var d2 = r.LengthSquared();
        if (d2 < 1e-24)
            return Vector3.Zero;

        var invD = 1.0 / global::System.Math.Sqrt(d2);
        return r.Multiply(invD * (mass * gm / d2));
    }

    private sealed record ForceCompareRow(string Label, double Expected, double Actual, double AbsError);
}
