using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Numerics;

[Obsolete("Use System.Numerics.Vector3. Novolis.Physics.Numerics is retired.")]
public readonly struct Vector3d
{
    public Vector3d(double x, double y, double z) => Value = new((float)x, (float)y, (float)z);
    public Vector3 Value { get; }
    public static implicit operator Vector3(Vector3d v) => v.Value;
}

[Obsolete("Use System.Numerics.Quaternion. Novolis.Physics.Numerics is retired.")]
public readonly struct Quaterniond
{
    public Quaterniond(double x, double y, double z, double w) => Value = new((float)x, (float)y, (float)z, (float)w);
    public Quaternion Value { get; }
    public static implicit operator Quaternion(Quaterniond q) => q.Value;
}

[Obsolete("Use Novolis.Math.Geometry.Ray3.")]
public readonly struct Ray3d(Ray3 value)
{
    public Ray3 Value { get; } = value;
    public static implicit operator Ray3(Ray3d r) => r.Value;
}

[Obsolete("Use Novolis.Math.Geometry.Sphere3.")]
public readonly struct Sphere3d(Sphere3 value)
{
    public Sphere3 Value { get; } = value;
    public static implicit operator Sphere3(Sphere3d s) => s.Value;
}
