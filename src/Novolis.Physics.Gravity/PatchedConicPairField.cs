using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Gravity;

/// <summary>Two-body patched conic lite: primary inside SOI, otherwise secondary point mass.</summary>
public readonly struct PatchedConicPairField
{
    /// <summary>PatchedConicPairField operation.</summary>
    public PatchedConicPairField(
        Vector3 primaryPosition,
        double primaryGm,
        double primarySphereOfInfluenceRadius,
        Vector3 secondaryPosition,
        double secondaryGm)
    {
        PrimaryPosition = primaryPosition;
        PrimaryGm = primaryGm;
        PrimarySphereOfInfluenceRadius = primarySphereOfInfluenceRadius;
        SecondaryPosition = secondaryPosition;
        SecondaryGm = secondaryGm;
    }
/// <summary>PrimaryPosition.</summary>

    public Vector3 PrimaryPosition { get; }
    /// <summary>SecondaryPosition.</summary>
    public double PrimaryGm { get; }
    /// <summary>PrimarySphereOfInfluenceRadius.</summary>
    public double PrimarySphereOfInfluenceRadius { get; }
    /// <summary>SecondaryGm.</summary>
    public Vector3 SecondaryPosition { get; }
    /// <summary>SecondaryGm.</summary>
    public double SecondaryGm { get; }
}
