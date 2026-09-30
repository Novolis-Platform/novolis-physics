using System.Numerics;
using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Cloth;

/// <summary>Result of a cloth topology cut.</summary>
public readonly struct ClothCutResult(int severedJointCount, int remainingJointCount)
{
    /// <summary>How many distance joints were removed.</summary>
    public int SeveredJointCount { get; } = severedJointCount;

    /// <summary>Joints still active after the cut.</summary>
    public int RemainingJointCount { get; } = remainingJointCount;
}
