using System.Numerics;

namespace Novolis.Physics.Motion;

/// <summary>Quaternion integration helpers (BCL <see cref="Quaternion"/> only).</summary>
public static class QuaternionAngularIntegration
{
    /// <summary>First-order quaternion integration: q̇ = ½ q ⊗ (0,ω).</summary>
    public static Quaternion IntegrateAngularVelocity(Quaternion q, Vector3 omega, float dt)
    {
        var wq = new Quaternion(omega.X, omega.Y, omega.Z, 0f);
        var qDot = Scale(Multiply(q, wq), 0.5f);
        var next = new Quaternion(
            q.X + qDot.X * dt,
            q.Y + qDot.Y * dt,
            q.Z + qDot.Z * dt,
            q.W + qDot.W * dt);
        return Quaternion.Normalize(next);
    }

    public static Vector3 InverseRotate(Vector3 world, Quaternion orientation)
    {
        var inv = Quaternion.Inverse(orientation);
        return Vector3.Transform(world, inv);
    }

    private static Quaternion Multiply(Quaternion a, Quaternion b) =>
        new(
            a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
            a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
            a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
            a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);

    private static Quaternion Scale(Quaternion q, float s) => new(q.X * s, q.Y * s, q.Z * s, q.W * s);
}
