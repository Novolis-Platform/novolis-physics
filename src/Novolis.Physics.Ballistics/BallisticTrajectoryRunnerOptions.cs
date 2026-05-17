namespace Novolis.Physics.Ballistics;

public sealed class BallisticTrajectoryRunnerOptions
{
    public ProjectileTerrainStepOptions Step { get; init; } = new();

    public int MaxTrailPoints { get; init; } = int.MaxValue;

    public bool RecordTrail { get; init; } = true;
}
