using System.Numerics;

namespace Novolis.Physics.Ballistics;

/// <summary>Recorded impact event from terrain or mesh contact during ballistic stepping.</summary>
public readonly struct ProjectileTerrainImpact
{
    /// <summary>Impact position in world space (meters).</summary>
    public Vector3 Position { get; init; }

    /// <summary>Velocity at impact (m/s).</summary>
    public Vector3 Velocity { get; init; }

    /// <summary>Simulation time at impact (seconds).</summary>
    public double TimeSeconds { get; init; }

    /// <summary>Horizontal range from launch in the XZ plane (meters).</summary>
    public float HorizontalRangeMeters { get; init; }

    /// <summary>Speed at impact (m/s).</summary>
    public float ImpactSpeedMps { get; init; }

    /// <summary>Why the trajectory stopped.</summary>
    public ProjectileTerrainImpactReason Reason { get; init; }
}
