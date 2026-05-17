using System.Numerics;
using Novolis.Physics.Abstractions;

namespace Novolis.Physics.Ballistics;

/// <summary>Interpolates impact state along a swept segment from a mesh hit.</summary>
public static class SegmentImpactInterpolator
{
    public static Vector3 PositionAlongSegment(Vector3 segmentStart, Vector3 displacement, in HitInfo hit)
    {
        var travel = displacement.Length();
        if (travel < 1e-8f)
            return segmentStart;

        var frac = (float)(hit.Distance / travel);
        return segmentStart + displacement * frac;
    }

    public static double TimeAlongStep(double stepStartTime, double stepDt, float traveledBeforeHit, float chunkLength, float travelInStep)
    {
        if (travelInStep < 1e-8f)
            return stepStartTime;

        return stepStartTime + stepDt * (traveledBeforeHit + chunkLength) / travelInStep;
    }
}
