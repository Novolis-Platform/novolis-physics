using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Abstractions;

/// <summary>World-space force (N) and torque (N·m) from one effect; summed before integration.</summary>
/// <param name="Force">Linear force in world space (newtons).</param>
/// <param name="Torque">Torque in world space (newton-meters).</param>
public readonly record struct ForceSample(Vector3 Force, Vector3 Torque)
{
    /// <summary>Zero force and torque sample.</summary>
    public static ForceSample Zero => new(Vector3.Zero, Vector3.Zero);

    /// <summary>Component-wise sum of two force samples.</summary>
    public static ForceSample operator +(ForceSample a, ForceSample b) =>
        new(a.Force + b.Force, a.Torque + b.Torque);

    /// <summary>Scales force and torque by a scalar factor.</summary>
    public static ForceSample operator *(ForceSample a, double s) => new(a.Force.Multiply(s), a.Torque.Multiply(s));
}
