namespace Novolis.Physics.Ballistics;

/// <summary>
/// Piecewise-linear drag coefficient against Mach number.
/// <see cref="BoatTail"/> is an in-repo educational fit: a mild subsonic value, a transonic rise, and a supersonic decline.
/// It is not a firing table and it is not a copied coefficient table.
/// </summary>
public sealed class DragCoefficientLaw
{
    private readonly (double Mach, double Cd)[] _knots;

    /// <summary>Creates a law from Mach/Cd knots. Mach values must be strictly increasing.</summary>
    public DragCoefficientLaw(IReadOnlyList<(double Mach, double Cd)> knots)
    {
        if (knots is null || knots.Count < 2)
            throw new ArgumentException("A drag law needs at least two knots.", nameof(knots));

        _knots = new (double Mach, double Cd)[knots.Count];
        for (var i = 0; i < knots.Count; i++)
        {
            if (i > 0 && knots[i].Mach <= knots[i - 1].Mach)
                throw new ArgumentException("Mach knots must be strictly increasing.", nameof(knots));

            _knots[i] = knots[i];
        }
    }

    /// <summary>Educational boat-tail curve used by the artillery lab.</summary>
    public static DragCoefficientLaw BoatTail { get; } = new(
    [
        (0.0, 0.17),
        (0.80, 0.19),
        (1.00, 0.32),
        (1.15, 0.41),
        (1.40, 0.34),
        (2.00, 0.28),
        (3.00, 0.25),
    ]);

    /// <summary>Interpolates Cd. Mach outside the knots clamps to the end values.</summary>
    public double Evaluate(double mach)
    {
        if (double.IsNaN(mach) || mach <= _knots[0].Mach)
            return _knots[0].Cd;

        var last = _knots[^1];
        if (mach >= last.Mach)
            return last.Cd;

        for (var i = 1; i < _knots.Length; i++)
        {
            if (mach > _knots[i].Mach)
                continue;

            var left = _knots[i - 1];
            var right = _knots[i];
            var t = (mach - left.Mach) / (right.Mach - left.Mach);
            return left.Cd + t * (right.Cd - left.Cd);
        }

        return last.Cd;
    }
}
