namespace Novolis.Physics.Ballistics;

/// <summary>Fixed-step settings for terrain-aware projectile integration.</summary>
public sealed class ProjectileTerrainStepOptions
{
    /// <summary>Integration timestep (seconds).</summary>
    public double DtSeconds { get; init; } = 1.0 / 120.0;

    /// <summary>Projectile collision radius for sweeps (meters).</summary>
    public float ProjectileRadius { get; init; } = 0.08f;

    /// <summary>Maximum segment length per mesh sweep (meters).</summary>
    public float MaxSweepMeters { get; init; } = 1.5f;

    /// <summary>Safety cap on integration steps per trajectory.</summary>
    public int MaxSteps { get; init; } = 120_000;
}
