using Novolis.Physics.TestSupport;
using Novolis.Physics.Ballistics;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

/// <summary>Vacuum range error vs dt — exposes integrator scaling mistakes.</summary>
public sealed class BallisticDtSweepTests
{
    private const double G = 9.80665;

    [Test]
    public async Task VacuumRange_ErrorDecreasesWithSmallerDt_Table()
    {
        const double speed = 50.0;
        const double angle = 35.0 * global::System.Math.PI / 180.0;
        var vx = speed * global::System.Math.Cos(angle);
        var vy = speed * global::System.Math.Sin(angle);
        var analyticalRange = 2.0 * vx * vy / G;
        var dts = new[] { 1.0 / 30.0, 1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0 };
        var rows = new List<DtSweepRow>();
        foreach (var dt in dts)
        {
            var sim = new ProjectileBallisticSimulation(null);
            var env = new ProjectileBallisticEnvironment(G, 0);
            var state = new ProjectileState(Vector3.Zero, PhysicsTestVectors.V(vx, vy, 0), massKg: 1.0, 0);
            var prev = state;
            while (state.Position.Y >= 0)
            {
                prev = state;
                state = sim.Step(state, dt, env);
            }

            var impact = ProjectileMath.InterpolateGroundImpact(prev, state);
            rows.Add(new DtSweepRow(dt, impact.Position.X, analyticalRange, global::System.Math.Abs(impact.Position.X - analyticalRange)));
        }

        var o = NovolisPhysicsTestTrace.Out;
        PhysicsDashboard.SectionAndTable(
            o,
            "Ballistic dt sweep (vacuum, v=50 m/s, 35 deg)",
            rows,
            new TableOptions { MaxCellWidth = 24, RightAlignNumericColumns = true },
            tableCaption: FormattableString.Invariant($"analytical range = {analyticalRange:G9} m"));

        await Assert.That(rows[^1].AbsErrorM).IsLessThanOrEqualTo((float)(rows[0].AbsErrorM * 1.05));
        await Assert.That(rows[^1].AbsErrorM).IsLessThanOrEqualTo((float)(0.02 * global::System.Math.Abs(analyticalRange)));
    }

    private sealed record DtSweepRow(double DtSeconds, double SimulatedRangeM, double AnalyticalRangeM, double AbsErrorM);
}
