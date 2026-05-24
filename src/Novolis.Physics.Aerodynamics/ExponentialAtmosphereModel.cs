namespace Novolis.Physics.Aerodynamics;

/// <summary>ρ(h) = ρ₀ · exp(-h / H).</summary>
public sealed class ExponentialAtmosphereModel : IAtmosphereModel
{
    /// <summary>ExponentialAtmosphereModel operation.</summary>
    public ExponentialAtmosphereModel(double seaLevelDensityKgPerM3, double scaleHeightMeters)
    {
        SeaLevelDensity = seaLevelDensityKgPerM3;
        ScaleHeightMeters = scaleHeightMeters;
    }
/// <summary>SeaLevelDensity.</summary>

    public double SeaLevelDensity { get; }
    /// <summary>DensityAtAltitude operation.</summary>
    public double ScaleHeightMeters { get; }

    /// <summary>DensityAtAltitude operation.</summary>
    public double DensityAtAltitude(double altitudeMeters) =>
        SeaLevelDensity * global::System.Math.Exp(-altitudeMeters / ScaleHeightMeters);
}
