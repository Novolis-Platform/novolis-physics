namespace Novolis.Physics.Cloth;

/// <summary>Grid layout and constraint stiffnesses for <see cref="ClothSheetPreset"/>.</summary>
public sealed class ClothSheetOptions
{
    /// <summary>Number of particles along the sheet width (columns). Minimum 2.</summary>
    public int Columns { get; set; } = 12;

    /// <summary>Number of particles along the sheet height (rows). Minimum 2.</summary>
    public int Rows { get; set; } = 10;

    /// <summary>Rest spacing between adjacent particles (meters).</summary>
    public float Spacing { get; set; } = 0.18f;

    /// <summary>Stiffness of structural (edge) distance joints in 0..1.</summary>
    public float StructuralStiffness { get; set; } = 1f;

    /// <summary>When true, add diagonal shear joints.</summary>
    public bool IncludeShear { get; set; } = true;

    /// <summary>Stiffness of shear joints in 0..1.</summary>
    public float ShearStiffness { get; set; } = 0.85f;

    /// <summary>When true, add bend joints skipping one particle.</summary>
    public bool IncludeBend { get; set; } = true;

    /// <summary>Stiffness of bend joints in 0..1.</summary>
    public float BendStiffness { get; set; } = 0.4f;

    /// <summary>Which particles are world-anchored after each solve.</summary>
    public ClothPinMode PinMode { get; set; } = ClothPinMode.TopRow;
}
