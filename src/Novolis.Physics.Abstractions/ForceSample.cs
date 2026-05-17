using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Abstractions;

/// <summary>World-space force (N) and torque (N·m) from one effect; summed before integration.</summary>
public readonly record struct ForceSample(Vector3 Force, Vector3 Torque)
{
    public static ForceSample Zero => new(Vector3.Zero, Vector3.Zero);

    public static ForceSample operator +(ForceSample a, ForceSample b) =>
        new(a.Force + b.Force, a.Torque + b.Torque);

    public static ForceSample operator *(ForceSample a, double s) => new(a.Force.Multiply(s), a.Torque.Multiply(s));
}
