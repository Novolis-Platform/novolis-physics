namespace Novolis.Physics.Ballistics;

/// <summary>Mass, reference area, and drag coefficient for quadratic drag models.</summary>
public readonly struct ProjectileProfile
{
    /// <summary>ProjectileProfile operation.</summary>
    public ProjectileProfile(double massKg, double referenceAreaM2, double dragCoefficient)
    {
        MassKg = massKg;
        ReferenceAreaM2 = referenceAreaM2;
        DragCoefficient = dragCoefficient;
    }
/// <summary>MassKg.</summary>

    public double MassKg { get; }
    /// <summary>ReferenceAreaM2.</summary>
    public double ReferenceAreaM2 { get; }
    /// <summary>DragCoefficient.</summary>
    public double DragCoefficient { get; }
}
