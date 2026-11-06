using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Ballistics;

/// <summary>
/// Semi-implicit ballistic integration in 3D with uniform <c>−Y</c> gravity and optional quadratic drag.
/// Use <c>Z = 0</c> for planar cannon-style problems (range along +X, height +Y).
/// The frame is East-Up-North when Coriolis is enabled: +X east, +Y up, +Z north.
/// Without <see cref="DragCoefficientLaw"/>, drag uses <see cref="ProjectileProfile.DragCoefficient"/>.
/// </summary>
public sealed class ProjectileBallisticSimulation
{
    private readonly ProjectileSemiImplicitIntegrator _integrator = new();
    private readonly ProjectileProfile? _dragProfile;
    private readonly DragCoefficientLaw? _machDrag;

    /// <summary>Constant-Cd drag when <paramref name="dragProfile"/> is set. Pass <paramref name="machDrag"/> to replace that Cd with a Mach curve.</summary>
    public ProjectileBallisticSimulation(ProjectileProfile? dragProfile = null, DragCoefficientLaw? machDrag = null)
    {
        _dragProfile = dragProfile;
        _machDrag = machDrag;
    }

    /// <summary>Integrates one step. Coriolis is applied only when <see cref="ProjectileBallisticEnvironment.LatitudeDegrees"/> is finite.</summary>
    public ProjectileState Step(ProjectileState state, double dtSeconds, ProjectileBallisticEnvironment environment)
    {
        var gravity = new Vector3(0f, (float)(-state.MassKg * environment.GravityMetersPerSecondSquared), 0f);
        var extra = Vector3.Zero;
        if (_dragProfile is { } profile && environment.AirDensityKgPerM3 > 1e-30)
        {
            var vRel = state.Velocity - environment.WindMetersPerSecond;
            var speed = vRel.Length();
            if (speed > 1e-9)
            {
                var cd = profile.DragCoefficient;
                if (_machDrag is not null && environment.TemperatureKelvin > 1.0)
                {
                    var sound = StandardAtmosphere.SpeedOfSound(environment.TemperatureKelvin);
                    if (sound > 1.0)
                        cd = _machDrag.Evaluate(speed / sound);
                }

                var q = 0.5 * environment.AirDensityKgPerM3 * cd * profile.ReferenceAreaM2 * speed * speed;
                extra = (vRel / speed).Multiply(-q);
            }
        }

        if (!double.IsNaN(environment.LatitudeDegrees))
        {
            var accel = CoriolisAcceleration.EastUpNorth(environment.LatitudeDegrees, state.Velocity);
            extra += accel.Multiply(state.MassKg);
        }

        var next = _integrator.Step(state, new ForceSample(gravity + extra, Vector3.Zero), dtSeconds);
        return next with { TimeSeconds = state.TimeSeconds + dtSeconds };
    }
}
