using Novolis.Physics.TestSupport;
using System.Globalization;
using System.Reflection;
using System.Text;

namespace Novolis.Physics.TestSupport;

/// <summary>Layout and safety limits for <see cref="TestOutput.Table{T}"/>.</summary>
public sealed class TableOptions
{
    /// <summary>Hard cap on rendered cell text (after whitespace flattening).</summary>
    public int MaxCellWidth { get; init; } = 48;

    public int MinCellWidth { get; init; } = 3;

    /// <summary>Maximum data rows (excluding header). Extra source rows are omitted with a footnote.</summary>
    public int MaxRows { get; init; } = 200;

    public string Ellipsis { get; init; } = "...";

    /// <summary>Replace CR/LF/tab runs with a single space so each table row stays one console line.</summary>
    public bool FlattenWhitespaceInCells { get; init; } = true;

    /// <summary>When true, body cells that parse as invariant doubles are padded left (headers stay left-padded).</summary>
    public bool RightAlignNumericColumns { get; init; } = false;
}
