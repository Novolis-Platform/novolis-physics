using System.Numerics;
using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Cloth;

/// <summary>
/// Radial fragmentation query — same sever pipeline as a blade, sized for blasts.
/// Impulse is applied separately via <see cref="ClothCutOps.ApplyBlastImpulse"/>.
/// </summary>
public readonly struct ClothBlast(Vector3 epicenter, float radius, float impulseSpeed = 0f)
{
    /// <summary>Blast center in world space.</summary>
    public Vector3 Epicenter { get; } = epicenter;

    /// <summary>Joints with midpoint inside this radius are severed.</summary>
    public float Radius { get; } = System.Math.Max(1e-4f, radius);

    /// <summary>Optional outward speed added to nearby free particles (m/s).</summary>
    public float ImpulseSpeed { get; } = System.Math.Max(0f, impulseSpeed);
}
