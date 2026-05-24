using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Abstractions;

/// <summary>Diagonal inertia in body space; torque in world space is mapped to body for integration.</summary>
public readonly struct RigidBodyState
{
    /// <summary>Initializes rigid-body kinematic and inertial state.</summary>
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

    /// <summary>World-space center of mass (meters).</summary>
    public Vector3 Position { get; init; }

    /// <summary>World-space linear velocity (m/s).</summary>
    public Vector3 Velocity { get; init; }

    /// <summary>Body-to-world orientation (normalized).</summary>
    public Quaternion Orientation { get; init; }

    /// <summary>Angular velocity in body space (rad/s).</summary>
    public Vector3 AngularVelocity { get; init; }

    /// <summary>Mass (kilograms).</summary>
    public double Mass { get; init; }

    /// <summary>Principal moments of inertia in body axes (kg·m²).</summary>
    public Vector3 InertiaDiagonalBody { get; init; }
}
