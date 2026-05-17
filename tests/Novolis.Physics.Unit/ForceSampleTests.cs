using Novolis.Physics.TestSupport;
using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

[NotInParallel(NovolisPhysicsTestTrace.NotInParallelKey)]
public sealed class ForceSampleTests
{
    [Test]
    public async Task ForceSample_Addition_Aggregates()
    {
        var a = new ForceSample(PhysicsTestVectors.V(1, 0, 0), Vector3.Zero);
        var b = new ForceSample(PhysicsTestVectors.V(0, 2, 0), PhysicsTestVectors.V(0, 0, 3));
        var s = a + b;

        var o = NovolisPhysicsTestTrace.Out;
        o.Section("ForceSample addition (compact)");
        o.Line("a.Force, a.Torque", "(1,0,0), (0,0,0)");
        o.Line("b.Force, b.Torque", "(0,2,0), (0,0,3)");

        o.Results("ForceSample addition - outcome");
        o.Table(
            new[]
            {
                new WrenchRow("a", a.Force.X, a.Force.Y, a.Force.Z, a.Torque.X, a.Torque.Y, a.Torque.Z),
                new WrenchRow("b", b.Force.X, b.Force.Y, b.Force.Z, b.Torque.X, b.Torque.Y, b.Torque.Z),
                new WrenchRow("sum", s.Force.X, s.Force.Y, s.Force.Z, s.Torque.X, s.Torque.Y, s.Torque.Z),
            },
            new TableOptions { RightAlignNumericColumns = true },
            caption: "Force (N) and torque (N·m); expect sum force (1,2,0), sum torque (0,0,3)");

        await Assert.That(s.Force).IsEqualTo(PhysicsTestVectors.V(1, 2, 0));
        await Assert.That(s.Torque).IsEqualTo(PhysicsTestVectors.V(0, 0, 3));
    }

    private sealed record WrenchRow(string Label, double Fx, double Fy, double Fz, double Tx, double Ty, double Tz);
}
