using Novolis.Physics.Astro;

using TUnit.Core;

namespace Novolis.Physics.Unit.Astro;

public sealed class AstronomicalUnitsTests
{
    [Test]
    public async Task Ly_RoundTrip()
    {
        var meters = AstronomicalUnits.LyToMeters(1.0);
        var ly = AstronomicalUnits.MetersToLy(meters);
        await Assert.That(global::System.Math.Abs(ly - 1.0)).IsLessThan(1e-12);
    }

    [Test]
    public async Task Pc_And_Au_Positive()
    {
        await Assert.That(AstronomicalUnits.PcToMeters(1)).IsGreaterThan(AstronomicalUnits.LyToMeters(1));
        await Assert.That(AstronomicalUnits.AuToMeters(1)).IsGreaterThan(0);
        var v = AstronomicalUnits.LightYearsToVector3(1, 0, 0);
        await Assert.That(v.X).IsGreaterThan(0);
    }
}
