using System.Numerics;
using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Cloth;

/// <summary>
/// Finite cutting edge for cloth (sword, knife, laser). Half-thickness is the
/// capture radius around the heel→tip segment; joints whose particle-pair segments
/// come within that radius are candidates for severing.
/// </summary>
public readonly struct ClothBlade(Vector3 heel, Vector3 tip, float halfThickness = 0.05f)
{
    /// <summary>Blade root / hilt-side point.</summary>
    public Vector3 Heel { get; } = heel;

    /// <summary>Blade tip.</summary>
    public Vector3 Tip { get; } = tip;

    /// <summary>Half-thickness of the cutting volume (meters).</summary>
    public float HalfThickness { get; } = System.Math.Max(1e-4f, halfThickness);

    /// <summary>Blade length.</summary>
    public float Length => Vector3.Distance(Heel, Tip);
}

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

/// <summary>Result of a cloth topology cut.</summary>
public readonly struct ClothCutResult(int severedJointCount, int remainingJointCount)
{
    /// <summary>How many distance joints were removed.</summary>
    public int SeveredJointCount { get; } = severedJointCount;

    /// <summary>Joints still active after the cut.</summary>
    public int RemainingJointCount { get; } = remainingJointCount;
}
