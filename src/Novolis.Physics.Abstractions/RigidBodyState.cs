using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Abstractions;

/// <summary>Diagonal inertia in body space; torque in world space is mapped to body for integration.</summary>
public readonly struct RigidBodyState
{
    public RigidBodyState(
        Vector3 position,
        Vector3 velocity,
        Quaternion orientation,
        Vector3 angularVelocity,
        double mass,
        Vector3 inertiaDiagonalBody)
    {
        Position = position;
        Velocity = velocity;
        Orientation = Quaternion.Normalize(orientation);
        AngularVelocity = angularVelocity;
        Mass = mass;
        InertiaDiagonalBody = inertiaDiagonalBody;
    }

    public Vector3 Position { get; init; }
    public Vector3 Velocity { get; init; }
    public Quaternion Orientation { get; init; }
    public Vector3 AngularVelocity { get; init; }
    public double Mass { get; init; }
    public Vector3 InertiaDiagonalBody { get; init; }
}
