using System.Numerics;
using Novolis.Math.Geometry;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Ballistics;

/// <summary>
/// Integrate-then-sweep ballistic step against mesh BVH and heightfield/range contact.
/// Displacement uses post-integration position minus start (matches <see cref="ProjectileSemiImplicitIntegrator"/>).
/// </summary>
public static class ProjectileTerrainStepper
{
    /// <summary>AdvanceOne operation.</summary>
    public static bool AdvanceOne(
        ref ProjectileState state,
        ProjectileBallisticSimulation simulation,
        ProjectileBallisticEnvironment environment,
        BvhStaticWorld? collisionWorld,
        IProjectileTerrainContact terrain,
        in ProjectileTerrainStepOptions options,
        Vector3 rangeOrigin,
        out ProjectileTerrainImpact? impact)
    {
        impact = null;
        var dt = options.DtSeconds;
        var radius = options.ProjectileRadius;
        var maxSweep = options.MaxSweepMeters;

        var startPos = state.Position;
        var startVel = state.Velocity;
        var startTime = state.TimeSeconds;

        var candidate = simulation.Step(state, dt, environment);
        var displacement = candidate.Position - startPos;
        var dist = displacement.Length();

        if (TryRangeOrGroundHit(terrain, startPos, candidate.Position, startVel, startTime, dt, rangeOrigin, radius, out impact))
            return true;

        if (dist > maxSweep)
        {
            var splits = (int)System.Math.Ceiling(dist / maxSweep);
            var subDt = dt / splits;
            for (var s = 0; s < splits; s++)
            {
                var subStart = state.Position;
                var subCandidate = simulation.Step(state, subDt, environment);
                var subDisp = subCandidate.Position - subStart;

                if (TryRangeOrGroundHit(terrain, subStart, subCandidate.Position, state.Velocity, state.TimeSeconds, subDt, rangeOrigin, radius, out impact))
                    return true;

                if (TryTerrainHit(terrain, collisionWorld, ref state, subDisp, subDt, maxSweep, radius, rangeOrigin, out impact))
                    return true;

                state = subCandidate;
            }
        }
        else
        {
            if (TryTerrainHit(terrain, collisionWorld, ref state, displacement, dt, maxSweep, radius, rangeOrigin, out impact))
                return true;

            state = candidate;
        }

        if (TryFallbackContact(terrain, state, rangeOrigin, radius, out impact))
            return true;

        return false;
    }

    private static bool TryRangeOrGroundHit(
        IProjectileTerrainContact terrain,
        Vector3 from,
        Vector3 to,
        Vector3 velocity,
        double startTime,
        double stepDt,
        Vector3 rangeOrigin,
        float radius,
        out ProjectileTerrainImpact? impact)
    {
        impact = null;
        if (terrain.TrySegmentLeavesRange(from, to, out var boundaryHit, out var boundaryFrac))
        {
            var t = startTime + stepDt * boundaryFrac;
            var ground = terrain.ProjectOntoSurface(boundaryHit);
            impact = CreateImpact(ground, velocity, t, rangeOrigin, ProjectileTerrainImpactReason.BeyondRange);
            return true;
        }

        if (terrain.TryHeightfieldContact(to, radius))
        {
            impact = CreateImpact(to, velocity, startTime + stepDt, rangeOrigin, ProjectileTerrainImpactReason.Heightfield);
            return true;
        }

        return false;
    }

    private static bool TryFallbackContact(
        IProjectileTerrainContact terrain,
        ProjectileState state,
        Vector3 rangeOrigin,
        float radius,
        out ProjectileTerrainImpact? impact)
    {
        impact = null;
        var p = state.Position;
        if (!terrain.IsInside(p.X, p.Z))
        {
            impact = CreateImpact(
                terrain.ProjectOntoSurface(p),
                state.Velocity,
                state.TimeSeconds,
                rangeOrigin,
                ProjectileTerrainImpactReason.BeyondRange);
            return true;
        }

        if (terrain.TryHeightfieldContact(p, radius))
        {
            impact = CreateImpact(
                p,
                state.Velocity,
                state.TimeSeconds,
                rangeOrigin,
                ProjectileTerrainImpactReason.Heightfield);
            return true;
        }

        return false;
    }

    private static bool TryTerrainHit(
        IProjectileTerrainContact terrain,
        BvhStaticWorld? collisionWorld,
        ref ProjectileState state,
        Vector3 displacement,
        double stepDt,
        float maxSweepMeters,
        float radius,
        Vector3 rangeOrigin,
        out ProjectileTerrainImpact? impact)
    {
        impact = null;
        var travel = displacement.Length();
        if (travel < 1e-8f)
            return false;

        var dir = displacement / travel;
        var traveled = 0f;
        var startPos = state.Position;
        var startVel = state.Velocity;
        var startTime = state.TimeSeconds;

        while (traveled < travel - 1e-6f)
        {
            var chunkLen = MathF.Min(maxSweepMeters, travel - traveled);
            var chunk = dir * chunkLen;
            var segStart = startPos + dir * traveled;
            var segEnd = segStart + chunk;

            if (terrain.TrySegmentLeavesRange(segStart, segEnd, out var boundaryHit, out var boundaryFrac))
            {
                impact = CreateImpact(
                    terrain.ProjectOntoSurface(boundaryHit),
                    startVel,
                    startTime + stepDt * (traveled + chunkLen * boundaryFrac) / travel,
                    rangeOrigin,
                    ProjectileTerrainImpactReason.BeyondRange);
                return true;
            }

            if (terrain.TryHeightfieldContact(segEnd, radius))
            {
                impact = CreateImpact(
                    segEnd,
                    startVel,
                    startTime + stepDt * (traveled + chunkLen) / travel,
                    rangeOrigin,
                    ProjectileTerrainImpactReason.Heightfield);
                return true;
            }

            if (collisionWorld is not null)
            {
                var sphere = new Sphere(segStart, radius);
                if (BallisticsQueries.SweepProjectileSphere(collisionWorld, in sphere, chunk, out var hit))
                {
                    var impactPos = SegmentImpactInterpolator.PositionAlongSegment(segStart, chunk, in hit);
                    impact = CreateImpact(
                        impactPos,
                        startVel,
                        startTime + stepDt * (traveled + chunkLen * (float)(hit.Distance / chunkLen)) / travel,
                        rangeOrigin,
                        ProjectileTerrainImpactReason.TerrainMesh);
                    return true;
                }
            }

            traveled += chunkLen;
        }

        return false;
    }

    private static ProjectileTerrainImpact CreateImpact(
        Vector3 position,
        Vector3 velocity,
        double timeSeconds,
        Vector3 rangeOrigin,
        ProjectileTerrainImpactReason reason)
    {
        var horizontal = position - rangeOrigin;
        horizontal.Y = 0f;
        return new ProjectileTerrainImpact
        {
            Position = position,
            Velocity = velocity,
            TimeSeconds = timeSeconds,
            HorizontalRangeMeters = horizontal.Length(),
            ImpactSpeedMps = velocity.Length(),
            Reason = reason,
        };
    }
}
