using System.Numerics;

namespace Novolis.Physics.Abstractions;

/// <summary>Axis-aligned XZ range box from the origin.</summary>
public readonly struct AxisAlignedRangeBox
{
    /// <summary>Creates a square range box on the XZ plane from the origin.</summary>
    /// <param name="extentMeters">Maximum X and Z coordinate inside the box (meters).</param>
    public AxisAlignedRangeBox(float extentMeters)
        : this(extentMeters, extentMeters)
    {
    }

    /// <summary>Creates a rectangular range box on the XZ plane from the origin.</summary>
    public AxisAlignedRangeBox(float extentEastMeters, float extentNorthMeters)
    {
        if (extentEastMeters <= 0f)
            throw new ArgumentOutOfRangeException(nameof(extentEastMeters));
        if (extentNorthMeters <= 0f)
            throw new ArgumentOutOfRangeException(nameof(extentNorthMeters));

        ExtentEastMeters = extentEastMeters;
        ExtentNorthMeters = extentNorthMeters;
        ExtentMeters = MathF.Max(extentEastMeters, extentNorthMeters);
    }

    /// <summary>Legacy maximum extent (max of east and north).</summary>
    public float ExtentMeters { get; }

    /// <summary>Maximum X inside the box (meters).</summary>
    public float ExtentEastMeters { get; }

    /// <summary>Maximum Z inside the box (meters).</summary>
    public float ExtentNorthMeters { get; }

    /// <summary>Returns whether <paramref name="x"/> and <paramref name="z"/> lie inside the box.</summary>
    public bool IsInside(float x, float z) =>
        x >= 0f && x <= ExtentEastMeters && z >= 0f && z <= ExtentNorthMeters;

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
        if (MathF.Abs(delta.X) > 1e-8f)
        {
            if (delta.X > 0f && from.X <= ExtentEastMeters)
                TryPlane(from.X, ExtentEastMeters, delta.X, ref tBest, ref found);
            if (delta.X < 0f && from.X >= 0f)
                TryPlane(from.X, 0f, delta.X, ref tBest, ref found);
        }

        if (MathF.Abs(delta.Z) > 1e-8f)
        {
            if (delta.Z > 0f && from.Z <= ExtentNorthMeters)
                TryPlane(from.Z, ExtentNorthMeters, delta.Z, ref tBest, ref found);
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
