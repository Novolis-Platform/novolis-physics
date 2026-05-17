using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Ballistics;

/// <summary>Semi-implicit Euler for <see cref="ProjectileState"/> (no orientation).</summary>
public sealed class ProjectileSemiImplicitIntegrator : IIntegrator<ProjectileState>
{
    public ProjectileState Step(ProjectileState body, in ForceSample totalForcesAndTorques, double dtSeconds)
    {
        var invM = body.MassKg > 1e-30 ? 1.0 / body.MassKg : 0;
        var a = totalForcesAndTorques.Force.Multiply(invM);
        var v = body.Velocity + a.Multiply(dtSeconds);
        var p = body.Position + v.Multiply(dtSeconds);
        return new ProjectileState(p, v, body.MassKg, body.TimeSeconds);
    }
}
