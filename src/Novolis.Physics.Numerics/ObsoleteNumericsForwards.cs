using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Numerics;

/// <summary>Obsolete double-precision vector shim; use <see cref="Vector3"/>.</summary>
[Obsolete("Use System.Numerics.Vector3. Novolis.Physics.Numerics is retired.")]
public readonly struct Vector3d
{
    /// <summary>Initializes a new instance with double components cast to float.</summary>
    /// <param name="x">X component (meters).</param>
    /// <param name="y">Y component (meters).</param>
    /// <param name="z">Z component (meters).</param>
    public Vector3d(double x, double y, double z) => Value = new((float)x, (float)y, (float)z);

    /// <summary>Underlying single-precision vector.</summary>
    public Vector3 Value { get; }

    /// <summary>Implicit conversion to <see cref="Vector3"/>.</summary>
    public static implicit operator Vector3(Vector3d v) => v.Value;
}

/// <summary>Obsolete double-precision quaternion shim; use <see cref="Quaternion"/>.</summary>
[Obsolete("Use System.Numerics.Quaternion. Novolis.Physics.Numerics is retired.")]
public readonly struct Quaterniond
{
    /// <summary>Initializes a new instance with double components cast to float.</summary>
    public Quaterniond(double x, double y, double z, double w) => Value = new((float)x, (float)y, (float)z, (float)w);

    /// <summary>Underlying single-precision quaternion.</summary>
    public Quaternion Value { get; }

    /// <summary>Implicit conversion to <see cref="Quaternion"/>.</summary>
    public static implicit operator Quaternion(Quaterniond q) => q.Value;
}

/// <summary>Obsolete ray shim; use <see cref="Ray3"/>.</summary>
[Obsolete("Use Novolis.Math.Geometry.Ray3.")]
public readonly struct Ray3d(Ray3 value)
{
    /// <summary>Underlying ray.</summary>
    public Ray3 Value { get; } = value;

    /// <summary>Implicit conversion to <see cref="Ray3"/>.</summary>
    public static implicit operator Ray3(Ray3d r) => r.Value;
}

/// <summary>Obsolete sphere shim; use <see cref="Sphere3"/>.</summary>
[Obsolete("Use Novolis.Math.Geometry.Sphere3.")]
public readonly struct Sphere3d(Sphere3 value)
{
    /// <summary>Underlying sphere.</summary>
    public Sphere3 Value { get; } = value;

    /// <summary>Implicit conversion to <see cref="Sphere3"/>.</summary>
    public static implicit operator Sphere3(Sphere3d s) => s.Value;
}
