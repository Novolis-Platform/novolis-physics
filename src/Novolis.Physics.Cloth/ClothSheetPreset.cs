using System.Numerics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;

namespace Novolis.Physics.Cloth;

/// <summary>
/// Builds a rectangular cloth particle grid from <see cref="SphereState"/> particles and
/// <see cref="DistanceJoint"/> structural / shear / bend constraints — the same primitives used by ragdolls.
/// </summary>
public static class ClothSheetPreset
{
    /// <summary>Flattens (column, row) to a particle index. Row 0 is the top edge.</summary>
    public static int Index(int column, int row, int columns) => row * columns + column;

    /// <summary>Total particles for the given grid size.</summary>
    public static int ParticleCount(int columns, int rows) => columns * rows;

    /// <summary>Triangle index count for a filled quad grid (2 triangles per cell).</summary>
    public static int TriangleIndexCount(int columns, int rows)
    {
        if (columns < 2 || rows < 2)
            return 0;
        return (columns - 1) * (rows - 1) * 6;
    }

    /// <summary>
    /// Fills a hanging sheet: <paramref name="origin"/> is the top-left particle,
    /// <paramref name="right"/> points along columns, <paramref name="down"/> along rows (usually −Y).
    /// </summary>
    public static void BuildHanging(
        Vector3 origin,
        Vector3 right,
        Vector3 down,
        ClothSheetOptions options,
        IList<SphereState> spheres,
        IList<DistanceJoint> joints,
        IList<int> pinIndices,
        IList<Vector3> pinAnchors)
    {
        ArgumentNullException.ThrowIfNull(options);
        ArgumentNullException.ThrowIfNull(spheres);
        ArgumentNullException.ThrowIfNull(joints);
        ArgumentNullException.ThrowIfNull(pinIndices);
        ArgumentNullException.ThrowIfNull(pinAnchors);

        var columns = System.Math.Max(2, options.Columns);
        var rows = System.Math.Max(2, options.Rows);
        var spacing = System.Math.Max(1e-4f, options.Spacing);

        var rightDir = right.LengthSquared() > 1e-12f ? Vector3.Normalize(right) : Vector3.UnitX;
        var downDir = down.LengthSquared() > 1e-12f ? Vector3.Normalize(down) : -Vector3.UnitY;

        spheres.Clear();
        joints.Clear();
        pinIndices.Clear();
        pinAnchors.Clear();

        for (var row = 0; row < rows; row++)
        {
            for (var col = 0; col < columns; col++)
            {
                var position = origin + rightDir * (col * spacing) + downDir * (row * spacing);
                spheres.Add(new SphereState(position, Vector3.Zero));
            }
        }

        AddStructural(joints, spheres, columns, rows, options.StructuralStiffness);

        if (options.IncludeShear)
            AddShear(joints, spheres, columns, rows, options.ShearStiffness);

        if (options.IncludeBend)
            AddBend(joints, spheres, columns, rows, options.BendStiffness);

        CollectPins(columns, rows, options.PinMode, pinIndices);
        foreach (var pin in pinIndices)
            pinAnchors.Add(spheres[pin].Position);
    }

    /// <summary>Restores pinned particles to their anchor positions and zeroes velocity.</summary>
    public static void ApplyPins(
        IList<SphereState> spheres,
        ReadOnlySpan<int> pinIndices,
        ReadOnlySpan<Vector3> pinAnchors)
    {
        var count = System.Math.Min(pinIndices.Length, pinAnchors.Length);
        for (var i = 0; i < count; i++)
        {
            var index = pinIndices[i];
            if ((uint)index >= (uint)spheres.Count)
                continue;

            var sphere = spheres[index];
            sphere.Position = pinAnchors[i];
            sphere.Velocity = Vector3.Zero;
            sphere.IsSleeping = true;
            spheres[index] = sphere;
        }
    }

    /// <summary>Writes counter-clockwise triangle indices for a filled cloth mesh (two tris per cell).</summary>
    public static int WriteTriangleIndices(int columns, int rows, Span<int> destination)
    {
        var needed = TriangleIndexCount(columns, rows);
        if (needed == 0 || destination.Length < needed)
            return 0;

        var write = 0;
        for (var row = 0; row < rows - 1; row++)
        {
            for (var col = 0; col < columns - 1; col++)
            {
                var i00 = Index(col, row, columns);
                var i10 = Index(col + 1, row, columns);
                var i01 = Index(col, row + 1, columns);
                var i11 = Index(col + 1, row + 1, columns);

                destination[write++] = i00;
                destination[write++] = i01;
                destination[write++] = i10;

                destination[write++] = i10;
                destination[write++] = i01;
                destination[write++] = i11;
            }
        }

        return write;
    }

    /// <summary>Allocates triangle indices for the grid.</summary>
    public static int[] CreateTriangleIndices(int columns, int rows)
    {
        var count = TriangleIndexCount(columns, rows);
        if (count == 0)
            return [];

        var indices = new int[count];
        WriteTriangleIndices(columns, rows, indices);
        return indices;
    }

    private static void CollectPins(int columns, int rows, ClothPinMode mode, IList<int> pinIndices)
    {
        switch (mode)
        {
            case ClothPinMode.TopRow:
                for (var col = 0; col < columns; col++)
                    pinIndices.Add(Index(col, 0, columns));
                break;
            case ClothPinMode.TopCorners:
                pinIndices.Add(Index(0, 0, columns));
                pinIndices.Add(Index(columns - 1, 0, columns));
                break;
            case ClothPinMode.Corners:
                pinIndices.Add(Index(0, 0, columns));
                pinIndices.Add(Index(columns - 1, 0, columns));
                pinIndices.Add(Index(0, rows - 1, columns));
                pinIndices.Add(Index(columns - 1, rows - 1, columns));
                break;
            case ClothPinMode.None:
            default:
                break;
        }
    }

    private static void AddStructural(
        IList<DistanceJoint> joints,
        IList<SphereState> spheres,
        int columns,
        int rows,
        float stiffness)
    {
        for (var row = 0; row < rows; row++)
        {
            for (var col = 0; col < columns; col++)
            {
                var i = Index(col, row, columns);
                if (col + 1 < columns)
                    Link(joints, spheres, i, Index(col + 1, row, columns), stiffness);
                if (row + 1 < rows)
                    Link(joints, spheres, i, Index(col, row + 1, columns), stiffness);
            }
        }
    }

    private static void AddShear(
        IList<DistanceJoint> joints,
        IList<SphereState> spheres,
        int columns,
        int rows,
        float stiffness)
    {
        for (var row = 0; row < rows - 1; row++)
        {
            for (var col = 0; col < columns - 1; col++)
            {
                var i00 = Index(col, row, columns);
                var i11 = Index(col + 1, row + 1, columns);
                var i10 = Index(col + 1, row, columns);
                var i01 = Index(col, row + 1, columns);
                Link(joints, spheres, i00, i11, stiffness);
                Link(joints, spheres, i10, i01, stiffness);
            }
        }
    }

    private static void AddBend(
        IList<DistanceJoint> joints,
        IList<SphereState> spheres,
        int columns,
        int rows,
        float stiffness)
    {
        for (var row = 0; row < rows; row++)
        {
            for (var col = 0; col < columns; col++)
            {
                var i = Index(col, row, columns);
                if (col + 2 < columns)
                    Link(joints, spheres, i, Index(col + 2, row, columns), stiffness);
                if (row + 2 < rows)
                    Link(joints, spheres, i, Index(col, row + 2, columns), stiffness);
            }
        }
    }

    private static void Link(
        IList<DistanceJoint> joints,
        IList<SphereState> spheres,
        int a,
        int b,
        float stiffness)
    {
        var rest = Vector3.Distance(spheres[a].Position, spheres[b].Position);
        joints.Add(new DistanceJoint(a, b, rest, System.Math.Clamp(stiffness, 0f, 1f)));
    }
}
