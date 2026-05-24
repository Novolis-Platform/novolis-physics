using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Aerodynamics;

/// <summary>Inputs for <see cref="SimpleLiftDragModel"/>: atmosphere, altitude, wind, and aero coefficients.</summary>
public readonly struct SimpleAeroEnvironment
{
    /// <summary>SimpleAeroEnvironment operation.</summary>
    public SimpleAeroEnvironment(
        IAtmosphereModel atmosphere,
        double altitudeMeters,
        Vector3 windWorld,
        double referenceAreaM2,
        double dragCoefficient,
        double liftCoefficient,
        Vector3 liftReferenceForwardWorld)
    {
        Atmosphere = atmosphere;
        AltitudeMeters = altitudeMeters;
        WindWorld = windWorld;
        ReferenceAreaM2 = referenceAreaM2;
        DragCoefficient = dragCoefficient;
        LiftCoefficient = liftCoefficient;
        LiftReferenceForwardWorld = liftReferenceForwardWorld.Normalized();
    }
/// <summary>Atmosphere.</summary>

    public IAtmosphereModel Atmosphere { get; }
    /// <summary>ReferenceAreaM2.</summary>
    public double AltitudeMeters { get; }
    /// <summary>LiftCoefficient.</summary>
    public Vector3 WindWorld { get; }
    /// <summary>ReferenceAreaM2.</summary>
    public double ReferenceAreaM2 { get; }
    /// <summary>LiftCoefficient.</summary>
    public double DragCoefficient { get; }
    /// <summary>LiftCoefficient.</summary>
    public double LiftCoefficient { get; }
    /// <summary>LiftReferenceForwardWorld.</summary>
    public Vector3 LiftReferenceForwardWorld { get; }
}
