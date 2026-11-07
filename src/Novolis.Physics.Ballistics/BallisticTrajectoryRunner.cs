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

    /// <summary>Initializes a runner with optional <paramref name="options"/>.</summary>
    public BallisticTrajectoryRunner(BallisticTrajectoryRunnerOptions? options = null)
    {
        Options = options ?? new BallisticTrajectoryRunnerOptions();
    }

    /// <summary>Trail, stepping, and recording options.</summary>
    public BallisticTrajectoryRunnerOptions Options { get; }

    /// <summary>Current flight phase.</summary>
    public BallisticTrajectoryPhase Phase { get; private set; } = BallisticTrajectoryPhase.Ready;

    /// <summary>Recorded world positions along the trajectory.</summary>
    public IReadOnlyList<Vector3> Trail => _trail;

    /// <summary>Impact details when <see cref="Phase"/> is <see cref="BallisticTrajectoryPhase.Impacted"/>.</summary>
    public ProjectileTerrainImpact? Impact { get; private set; }

    /// <summary>Current projectile position (meters).</summary>
    public Vector3 CurrentPosition => _state.Position;

    /// <summary>Current projectile velocity (m/s).</summary>
    public Vector3 CurrentVelocity => _state.Velocity;

    /// <summary>Elapsed simulation time (seconds).</summary>
    public double TimeSeconds => _state.TimeSeconds;

    /// <summary>Starts a new trajectory from <paramref name="start"/>.</summary>
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

    /// <summary>Clears trail and returns to <see cref="BallisticTrajectoryPhase.Ready"/>.</summary>
    public void Reset()
    {
        _trail.Clear();
        Impact = null;
        Phase = BallisticTrajectoryPhase.Ready;
        _stepCounter = 0;
    }

    /// <summary>Advances up to <paramref name="maxPhysicsSteps"/> while still in flight.</summary>
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

    /// <summary>Integrates one fixed step with terrain and optional mesh collision.</summary>
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
    /// <remarks>
    /// Always finishes on the heightfield or map edge when possible — do not leave the last
    /// sample hanging mid-air just because <paramref name="maxPoints"/> ran out near apogee.
    /// </remarks>
    public static IReadOnlyList<Vector3> BuildPreview(
        ProjectileBallisticSimulation simulation,
        ProjectileBallisticEnvironment environment,
        IProjectileTerrainContact terrain,
        ProjectileState start,
        double dtSeconds,
        double maxTimeSeconds,
        int maxPoints)
    {
        if (dtSeconds <= 0)
            dtSeconds = 1.0 / 30.0;

        var points = new List<Vector3>(System.Math.Min(System.Math.Max(64, maxPoints), 8_192)) { start.Position };
        var state = start;
        var finished = false;

        // Soft cap for the drawn polyline; keep integrating past it until ground/edge so MPI is real.
        var drawCap = System.Math.Max(8, maxPoints);
        var hardSteps = System.Math.Max(drawCap * 4, (int)System.Math.Ceiling(maxTimeSeconds / dtSeconds) + 8);

        for (var step = 0; step < hardSteps; step++)
        {
            var prev = state.Position;
            state = simulation.Step(state, dtSeconds, environment);
            var p = state.Position;

            if (terrain.TrySegmentLeavesRange(prev, p, out var boundaryHit, out _))
            {
                points.Add(terrain.ProjectOntoSurface(boundaryHit));
                finished = true;
                break;
            }

            if (!terrain.IsInside(p.X, p.Z))
            {
                points.Add(terrain.ProjectOntoSurface(p));
                finished = true;
                break;
            }

            if (terrain.TryHeightfieldContact(p, 0.08f))
            {
                points.Add(p);
                finished = true;
                break;
            }

            if (points.Count < drawCap)
                points.Add(p);
            else if (step % 3 == 0)
                points[^1] = p; // keep the tip advancing after the draw cap
        }

        if (!finished && points.Count >= 1)
            points[^1] = terrain.ProjectOntoSurface(state.Position);

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
