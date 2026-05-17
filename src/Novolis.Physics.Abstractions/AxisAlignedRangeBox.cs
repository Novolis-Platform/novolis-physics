using System.Numerics;

namespace Novolis.Physics.Abstractions;

/// <summary>Axis-aligned XZ range box from origin to <see cref="ExtentMeters"/> on both axes.</summary>
public readonly struct AxisAlignedRangeBox
{
    public AxisAlignedRangeBox(float extentMeters)
    {
        if (extentMeters <= 0f)
            throw new ArgumentOutOfRangeException(nameof(extentMeters));

        ExtentMeters = extentMeters;
    }

    public float ExtentMeters { get; }

    public bool IsInside(float x, float z) =>
        x >= 0f && x <= ExtentMeters && z >= 0f && z <= ExtentMeters;

    /// <summary>First exit of segment from the XZ range box. Returns false if both endpoints are inside.</summary>
    public bool TrySegmentLeavesRange(
        Vector3 from,
        Vector3 to,
        out Vector3 hitPoint,
        out float fractionAlongSegment)
    {
        hitPoint = default;
        fractionAlongSegment = 1f;

        if (IsInside(from.X, from.Z) && IsInside(to.X, to.Z))
            return false;

        var delta = to - from;
        var tBest = 1f;
        var found = false;
        var extent = ExtentMeters;

        if (MathF.Abs(delta.X) > 1e-8f)
        {
            if (delta.X > 0f && from.X <= extent)
                TryPlane(from.X, extent, delta.X, ref tBest, ref found);
            if (delta.X < 0f && from.X >= 0f)
                TryPlane(from.X, 0f, delta.X, ref tBest, ref found);
        }

        if (MathF.Abs(delta.Z) > 1e-8f)
        {
            if (delta.Z > 0f && from.Z <= extent)
                TryPlane(from.Z, extent, delta.Z, ref tBest, ref found);
            if (delta.Z < 0f && from.Z >= 0f)
                TryPlane(from.Z, 0f, delta.Z, ref tBest, ref found);
        }

        if (!found || tBest < 0f || tBest > 1f)
            return false;

        fractionAlongSegment = tBest;
        hitPoint = from + delta * tBest;
        return true;
    }

    private static void TryPlane(float start, float plane, float delta, ref float tBest, ref bool found)
    {
        var t = (plane - start) / delta;
        if (t < 0f || t > 1f || t >= tBest)
            return;

        tBest = t;
        found = true;
    }
}
