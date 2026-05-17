using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Aerodynamics;

/// <summary>Inputs for <see cref="SimpleLiftDragModel"/>: atmosphere, altitude, wind, and aero coefficients.</summary>
public readonly struct SimpleAeroEnvironment
{
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

    public IAtmosphereModel Atmosphere { get; }
    public double AltitudeMeters { get; }
    public Vector3 WindWorld { get; }
    public double ReferenceAreaM2 { get; }
    public double DragCoefficient { get; }
    public double LiftCoefficient { get; }
    public Vector3 LiftReferenceForwardWorld { get; }
}
