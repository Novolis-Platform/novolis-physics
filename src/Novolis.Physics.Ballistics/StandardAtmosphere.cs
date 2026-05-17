namespace Novolis.Physics.Ballistics;

/// <summary>Ideal-gas air density with optional water-vapor correction (educational, not WMO-certified).</summary>
public static class StandardAtmosphere
{
    private const double Rd = 287.058;
    private const double Rv = 461.495;

    /// <summary>Moist air density ρ = p_d/(R_d T) + p_v/(R_v T) with Buck saturation vapor pressure.</summary>
    public static double DensityKgPerM3(
        double pressurePa,
        double temperatureKelvin,
        double relativeHumidity01)
    {
        if (temperatureKelvin < 200.0 || pressurePa <= 0.0)
            return 0.0;

        var rh = global::System.Math.Clamp(relativeHumidity01, 0.0, 1.0);
        var tc = temperatureKelvin - 273.15;
        var es = 611.21 * global::System.Math.Exp((18.678 - tc / 234.5) * (tc / (257.14 + tc)));
        var pv = rh * es;
        var pd = global::System.Math.Max(0.0, pressurePa - pv);
        return (pd / (Rd * temperatureKelvin)) + (pv / (Rv * temperatureKelvin));
    }

    /// <summary>Barometric pressure vs geopotential altitude with linear temperature lapse from a reference level.</summary>
    public static double PressureAtAltitude(
        double referencePressurePa,
        double altitudeMeters,
        double referenceTemperatureKelvin,
        double lapseRateKelvinPerMeter = 0.0065)
    {
        const double g = 9.80665;
        const double molarMass = 0.0289644;
        const double r = 8.31447;

        if (altitudeMeters <= 0.0)
            return referencePressurePa;

        if (global::System.Math.Abs(lapseRateKelvinPerMeter) < 1e-12)
        {
            var scaleH = (r * referenceTemperatureKelvin) / (g * molarMass);
            return referencePressurePa * global::System.Math.Exp(-altitudeMeters / scaleH);
        }

        var t = referenceTemperatureKelvin - lapseRateKelvinPerMeter * altitudeMeters;
        if (t <= 0.0)
            return referencePressurePa * 0.5;

        var exponent = g * molarMass / (r * lapseRateKelvinPerMeter);
        return referencePressurePa * global::System.Math.Pow(t / referenceTemperatureKelvin, exponent);
    }
}
