namespace Novolis.Physics.Ballistics;

/// <summary>Options for <see cref="BallisticTrajectoryRunner"/> trail recording and stepping.</summary>
public sealed class BallisticTrajectoryRunnerOptions
{
    /// <summary>Per-step terrain and mesh contact settings.</summary>
    public ProjectileTerrainStepOptions Step { get; init; } = new();

    /// <summary>Maximum trail vertices retained (unbounded when <see cref="int.MaxValue"/>).</summary>
    public int MaxTrailPoints { get; init; } = int.MaxValue;

    /// <summary>When true, records positions into <see cref="BallisticTrajectoryRunner.Trail"/>.</summary>
    public bool RecordTrail { get; init; } = true;
}
