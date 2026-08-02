namespace Novolis.Physics.Cloth;

/// <summary>Which particles of a cloth sheet stay anchored in world space.</summary>
public enum ClothPinMode
{
    /// <summary>No pins; sheet is free-falling.</summary>
    None = 0,

    /// <summary>Entire top row (row 0) is pinned.</summary>
    TopRow = 1,

    /// <summary>Only the two top corners are pinned.</summary>
    TopCorners = 2,

    /// <summary>All four corner particles are pinned.</summary>
    Corners = 3,
}
