using System.Numerics;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Ballistics;

/// <summary>Stateful ballistic flight with trail buffer and impact recording.</summary>
public sealed class BallisticTrajectoryRunner
{
    private readonly List<Vector3> _trail = [];
    private ProjectileState _state;
    private int _stepCounter;

    public BallisticTrajectoryRunner(BallisticTrajectoryRunnerOptions? options = null)
    {
        Options = options ?? new BallisticTrajectoryRunnerOptions();
    }

    public BallisticTrajectoryRunnerOptions Options { get; }

    public BallisticTrajectoryPhase Phase { get; private set; } = BallisticTrajectoryPhase.Ready;

    public IReadOnlyList<Vector3> Trail => _trail;

    public ProjectileTerrainImpact? Impact { get; private set; }

    public Vector3 CurrentPosition => _state.Position;

    public Vector3 CurrentVelocity => _state.Velocity;

    public double TimeSeconds => _state.TimeSeconds;

    public void Begin(ProjectileState start)
    {
        _state = start;
        _trail.Clear();
        if (Options.RecordTrail)
            _trail.Add(start.Position);
        _stepCounter = 0;
        Impact = null;
        Phase = BallisticTrajectoryPhase.InFlight;
    }

    public void Reset()
    {
        _trail.Clear();
        Impact = null;
        Phase = BallisticTrajectoryPhase.Ready;
        _stepCounter = 0;
    }

    public void AdvanceWithBudget(
        ProjectileBallisticSimulation simulation,
        ProjectileBallisticEnvironment environment,
        BvhStaticWorld? collisionWorld,
        IProjectileTerrainContact terrain,
        Vector3 rangeOrigin,
        int maxPhysicsSteps)
    {
        for (var i = 0; i < maxPhysicsSteps && Phase == BallisticTrajectoryPhase.InFlight; i++)
            AdvanceOne(simulation, environment, collisionWorld, terrain, rangeOrigin);
    }

    public void AdvanceOne(
        ProjectileBallisticSimulation simulation,
        ProjectileBallisticEnvironment environment,
        BvhStaticWorld? collisionWorld,
        IProjectileTerrainContact terrain,
        Vector3 rangeOrigin)
    {
        if (Phase != BallisticTrajectoryPhase.InFlight)
            return;

        var env = environment;
        var hit = ProjectileTerrainStepper.AdvanceOne(
            ref _state,
            simulation,
            env,
            collisionWorld,
            terrain,
            Options.Step,
            rangeOrigin,
            out var impact);

        if (hit && impact is { } recorded)
        {
            if (Options.RecordTrail && (_trail.Count == 0 || _trail[^1] != recorded.Position))
                TryAddTrail(recorded.Position);
            Impact = recorded;
            Phase = BallisticTrajectoryPhase.Impacted;
            return;
        }

        _stepCounter++;
        if (Options.RecordTrail)
            TryAddTrail(_state.Position);

        if (_stepCounter >= Options.Step.MaxSteps)
        {
            Impact = new ProjectileTerrainImpact
            {
                Position = _state.Position,
                Velocity = _state.Velocity,
                TimeSeconds = _state.TimeSeconds,
                HorizontalRangeMeters = HorizontalRange(_state.Position, rangeOrigin),
                ImpactSpeedMps = _state.Velocity.Length(),
                Reason = ProjectileTerrainImpactReason.MaxSteps,
            };
            Phase = BallisticTrajectoryPhase.Impacted;
        }
    }

    /// <summary>Integrates a preview arc without mesh collision (heightfield + range only).</summary>
    public static IReadOnlyList<Vector3> BuildPreview(
        ProjectileBallisticSimulation simulation,
        ProjectileBallisticEnvironment environment,
        IProjectileTerrainContact terrain,
        ProjectileState start,
        double dtSeconds,
        double maxTimeSeconds,
        int maxPoints)
    {
        var points = new List<Vector3>(64) { start.Position };
        var state = start;

        for (var t = 0.0; t < maxTimeSeconds && points.Count < maxPoints; t += dtSeconds)
        {
            var prev = state.Position;
            state = simulation.Step(state, dtSeconds, environment);
            var p = state.Position;

            if (terrain.TrySegmentLeavesRange(prev, p, out var boundaryHit, out _))
            {
                points.Add(terrain.ProjectOntoSurface(boundaryHit));
                break;
            }

            if (!terrain.IsInside(p.X, p.Z))
            {
                points.Add(terrain.ProjectOntoSurface(p));
                break;
            }

            if (terrain.TryHeightfieldContact(p, 0.08f))
            {
                points.Add(p);
                break;
            }

            points.Add(p);
        }

        return points;
    }

    private void TryAddTrail(Vector3 point)
    {
        if (_trail.Count >= Options.MaxTrailPoints)
            return;

        _trail.Add(point);
    }

    private static float HorizontalRange(Vector3 position, Vector3 origin)
    {
        var horizontal = position - origin;
        horizontal.Y = 0f;
        return horizontal.Length();
    }
}
