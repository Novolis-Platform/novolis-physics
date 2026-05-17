using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Ballistics;

/// <summary>
/// Semi-implicit ballistic integration in 3D with uniform <c>−Y</c> gravity and optional quadratic drag.
/// Use <c>Z = 0</c> for planar cannon-style problems (range along +X, height +Y).
/// </summary>
public sealed class ProjectileBallisticSimulation
{
    private readonly ProjectileSemiImplicitIntegrator _integrator = new();
    private readonly ProjectileProfile? _dragProfile;

    public ProjectileBallisticSimulation(ProjectileProfile? dragProfile = null) => _dragProfile = dragProfile;

    public ProjectileState Step(ProjectileState state, double dtSeconds, ProjectileBallisticEnvironment environment)
    {
        var gravity = new Vector3(0f, (float)(-state.MassKg * environment.GravityMetersPerSecondSquared), 0f);
        var drag = Vector3.Zero;
        if (_dragProfile is { } profile && environment.AirDensityKgPerM3 > 1e-30)
        {
            var vRel = state.Velocity - environment.WindMetersPerSecond;
            var speed = vRel.Length();
            if (speed > 1e-9)
            {
                var q = 0.5 * environment.AirDensityKgPerM3 * profile.DragCoefficient * profile.ReferenceAreaM2 * speed * speed;
                drag = (vRel / speed).Multiply(-q);
            }
        }

        var next = _integrator.Step(state, new ForceSample(gravity + drag, Vector3.Zero), dtSeconds);
        return next with { TimeSeconds = state.TimeSeconds + dtSeconds };
    }
}
