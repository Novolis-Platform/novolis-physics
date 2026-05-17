using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Gravity;

/// <summary>Two-body patched conic lite: primary inside SOI, otherwise secondary point mass.</summary>
public readonly struct PatchedConicPairField
{
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

    public Vector3 PrimaryPosition { get; }
    public double PrimaryGm { get; }
    public double PrimarySphereOfInfluenceRadius { get; }
    public Vector3 SecondaryPosition { get; }
    public double SecondaryGm { get; }
}
