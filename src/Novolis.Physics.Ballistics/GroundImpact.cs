using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Ballistics;

/// <summary>Interpolated crossing of the <c>Y = 0</c> plane (descending).</summary>
public readonly struct GroundImpact(Vector3 position, double timeSeconds, Vector3 velocity)
{
    public Vector3 Position { get; } = position;
    public double TimeSeconds { get; } = timeSeconds;
    public Vector3 Velocity { get; } = velocity;

    public double ImpactSpeed => Velocity.Length();
}
