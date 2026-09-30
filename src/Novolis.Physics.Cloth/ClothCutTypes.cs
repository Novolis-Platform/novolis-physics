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
