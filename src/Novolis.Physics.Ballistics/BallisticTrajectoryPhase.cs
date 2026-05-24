namespace Novolis.Physics.Ballistics;

/// <summary>Lifecycle phase of a <see cref="BallisticTrajectoryRunner"/> session.</summary>
public enum BallisticTrajectoryPhase
{
    /// <summary>Initialized but not yet launched.</summary>
    Ready,

    /// <summary>Projectile is in flight.</summary>
    InFlight,

    /// <summary>Flight ended with a recorded impact.</summary>
    Impacted,
}
