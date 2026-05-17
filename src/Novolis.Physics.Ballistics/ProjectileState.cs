using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Ballistics;

/// <summary>Point-mass projectile: position, velocity, mass, and optional elapsed time.</summary>
public readonly struct ProjectileState
{
    public ProjectileState(Vector3 position, Vector3 velocity, double massKg, double timeSeconds = 0)
    {
        Position = position;
        Velocity = velocity;
        MassKg = massKg;
        TimeSeconds = timeSeconds;
    }

    public Vector3 Position { get; init; }
    public Vector3 Velocity { get; init; }
    public double MassKg { get; init; }

    /// <summary>Elapsed simulation time for this sample (caller advances per step).</summary>
    public double TimeSeconds { get; init; }
}
