namespace Novolis.Physics.Ballistics;

public sealed class ProjectileTerrainStepOptions
{
    public double DtSeconds { get; init; } = 1.0 / 120.0;

    public float ProjectileRadius { get; init; } = 0.08f;

    public float MaxSweepMeters { get; init; } = 1.5f;

    public int MaxSteps { get; init; } = 120_000;
}
