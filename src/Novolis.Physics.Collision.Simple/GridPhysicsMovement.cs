using System.Numerics;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Numerics;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Sphere sweep movement against an <see cref="IStaticWorld"/> (XZ motion, +Y up).</summary>
public static class GridPhysicsMovement
{
    /// <summary>
    /// Moves a sphere center on the XZ plane using separate axis sweeps.
    /// </summary>
    public static Vector2 TryMove(
        IStaticWorld world,
        Vector2 position,
        Vector2 delta,
        double radius,
        double centerY = 0.9)
    {
        if (delta.LengthSquared() < 1e-12)
            return position;

        var pos = new Vector3d(position.X, centerY, position.Y);
        var dx = new Vector3d(delta.X, 0, 0);
        if (System.Math.Abs(dx.X) > 1e-12)
        {
            var sphere = new Sphere3d(pos, radius);
            if (world.SweepSphere(sphere, dx, out var hit))
                pos = new Vector3d(pos.X + hit.Distance * System.Math.Sign(dx.X), pos.Y, pos.Z);
            else
                pos += dx;
        }

        var dz = new Vector3d(0, 0, delta.Y);
        if (System.Math.Abs(dz.Z) > 1e-12)
        {
            var sphere = new Sphere3d(pos, radius);
            if (world.SweepSphere(sphere, dz, out var hit))
                pos = new Vector3d(pos.X, pos.Y, pos.Z + hit.Distance * System.Math.Sign(dz.Z));
            else
                pos += dz;
        }

        return new Vector2((float)pos.X, (float)pos.Z);
    }
}
