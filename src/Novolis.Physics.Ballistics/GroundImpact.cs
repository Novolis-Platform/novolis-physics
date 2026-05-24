using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Ballistics;

/// <summary>Interpolated crossing of the <c>Y = 0</c> plane (descending).</summary>
public readonly struct GroundImpact(Vector3 position, double timeSeconds, Vector3 velocity)
{
    /// <summary>Position.</summary>
    public Vector3 Position { get; } = position;
    /// <summary>Velocity.</summary>
    public double TimeSeconds { get; } = timeSeconds;
    /// <summary>ImpactSpeed.</summary>
    public Vector3 Velocity { get; } = velocity;

    /// <summary>ImpactSpeed.</summary>
    public double ImpactSpeed => Velocity.Length();
}
