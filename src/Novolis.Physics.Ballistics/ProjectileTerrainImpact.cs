using System.Numerics;

namespace Novolis.Physics.Ballistics;

public readonly struct ProjectileTerrainImpact
{
    public Vector3 Position { get; init; }
    public Vector3 Velocity { get; init; }
    public double TimeSeconds { get; init; }
    public float HorizontalRangeMeters { get; init; }
    public float ImpactSpeedMps { get; init; }
    public ProjectileTerrainImpactReason Reason { get; init; }
}
