using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Gravity;

/// <summary>Environment: list of point masses with GM already combined (m³/s²).</summary>
public readonly struct PointMassField
{
    public PointMassField(ReadOnlyMemory<(Vector3 Position, double Gm)> sources)
    {
        Sources = sources;
    }

    public ReadOnlyMemory<(Vector3 Position, double Gm)> Sources { get; }
}
