using System.Numerics;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Ballistics;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class SegmentImpactInterpolatorTests
{
    [Test]
    public async Task PositionAlongSegment_ZeroTravel_ReturnsStart()
    {
        var p = SegmentImpactInterpolator.PositionAlongSegment(new Vector3(1, 2, 3), Vector3.Zero, new HitInfo(0, Vector3.Zero, Vector3.UnitY, 0));
        await Assert.That(p).IsEqualTo(new Vector3(1, 2, 3));
    }

    [Test]
    public async Task PositionAlongSegment_InterpolatesAlongDisplacement()
    {
        var start = Vector3.Zero;
        var disp = new Vector3(10, 0, 0);
        var hit = new HitInfo(4, new Vector3(4, 0, 0), Vector3.UnitY, 0);
        var p = SegmentImpactInterpolator.PositionAlongSegment(start, disp, in hit);
        await Assert.That(p.X).IsEqualTo(4f).Within(0.001f);
    }

    [Test]
    public async Task TimeAlongStep_ComputesFractionOfStep()
    {
        var t = SegmentImpactInterpolator.TimeAlongStep(1.0, 0.5, traveledBeforeHit: 2f, chunkLength: 1f, travelInStep: 10f);
        await Assert.That(t).IsEqualTo(1.15).Within(0.001);
        var zero = SegmentImpactInterpolator.TimeAlongStep(2.0, 0.1, 0, 1, 0);
        await Assert.That(zero).IsEqualTo(2.0);
    }
}
