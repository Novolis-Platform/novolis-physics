using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Uniform-grid sphere–sphere separation with SIMD distance tests and cell-centric pair loops.</summary>
public sealed class UniformGridSphereContactSolver
{
    private readonly Dictionary<int, List<int>> _cells = new();
    private readonly Stack<List<int>> _cellPool = new();

    public readonly struct Result
    {
        public int PairChecks { get; init; }
        public int Contacts { get; init; }
    }

    public Result Resolve(
        SphereSoA spheres,
        float radius,
        float gridCellSize,
        float restitution,
        bool applyImpulses,
        bool awakePairsOnly,
        float separationScale = 1.001f)
    {
        var minDiameter = radius * 2f * separationScale;
        var minDistSq = minDiameter * minDiameter;

        var pairChecks = 0;
        var contacts = 0;
        BuildGrid(spheres, gridCellSize);

        foreach (var (key, indices) in _cells)
        {
            var cx = key / 4096;
            var cz = key % 4096;
            contacts += ResolveCellPairs(
                spheres, indices, minDistSq, minDiameter, restitution, applyImpulses, awakePairsOnly, ref pairChecks);
            contacts += ResolveAcrossCells(
                spheres, cx, cz, 1, 0, minDistSq, minDiameter, restitution, applyImpulses, awakePairsOnly, ref pairChecks);
            contacts += ResolveAcrossCells(
                spheres, cx, cz, 0, 1, minDistSq, minDiameter, restitution, applyImpulses, awakePairsOnly, ref pairChecks);
            contacts += ResolveAcrossCells(
                spheres, cx, cz, 1, 1, minDistSq, minDiameter, restitution, applyImpulses, awakePairsOnly, ref pairChecks);
            contacts += ResolveAcrossCells(
                spheres, cx, cz, 1, -1, minDistSq, minDiameter, restitution, applyImpulses, awakePairsOnly, ref pairChecks);
        }

        return new Result { PairChecks = pairChecks, Contacts = contacts };
    }

    private int ResolveCellPairs(
        SphereSoA spheres,
        List<int> indices,
        float minDistSq,
        float minDiameter,
        float restitution,
        bool applyImpulses,
        bool awakePairsOnly,
        ref int pairChecks)
    {
        var contacts = 0;
        var n = indices.Count;
        for (var a = 0; a < n; a++)
        {
            var i = indices[a];
            var j0 = a + 1;
            if (j0 >= n)
                continue;

            contacts += ResolveSimdBlock(
                spheres,
                i,
                indices,
                j0,
                n,
                minDistSq,
                minDiameter,
                restitution,
                applyImpulses,
                awakePairsOnly,
                ref pairChecks);
        }

        return contacts;
    }

    private int ResolveAcrossCells(
        SphereSoA spheres,
        int cx,
        int cz,
        int dx,
        int dz,
        float minDistSq,
        float minDiameter,
        float restitution,
        bool applyImpulses,
        bool awakePairsOnly,
        ref int pairChecks)
    {
        if (!_cells.TryGetValue(CellKey(cx + dx, cz + dz), out var other) || other.Count == 0)
            return 0;

        if (!_cells.TryGetValue(CellKey(cx, cz), out var self) || self.Count == 0)
            return 0;

        var contacts = 0;
        for (var a = 0; a < self.Count; a++)
        {
            var i = self[a];
            for (var b = 0; b < other.Count; b++)
            {
                var j = other[b];
                pairChecks++;
                if (TrySeparate(
                        spheres,
                        i,
                        j,
                        minDistSq,
                        minDiameter,
                        restitution,
                        applyImpulses,
                        awakePairsOnly))
                    contacts++;
            }
        }

        return contacts;
    }

    private int ResolveSimdBlock(
        SphereSoA spheres,
        int i,
        List<int> indices,
        int jStart,
        int n,
        float minDistSq,
        float minDiameter,
        float restitution,
        bool applyImpulses,
        bool awakePairsOnly,
        ref int pairChecks)
    {
        var contacts = 0;
        var px = spheres.PosX[i];
        var py = spheres.PosY[i];
        var pz = spheres.PosZ[i];
        var minDistV = Vector128.Create(minDistSq);
        var pxV = Vector128.Create(px);
        var pyV = Vector128.Create(py);
        var pzV = Vector128.Create(pz);

        var j = jStart;
        for (; j + 3 < n; j += 4)
        {
            pairChecks += 4;
            var j0 = indices[j];
            var j1 = indices[j + 1];
            var j2 = indices[j + 2];
            var j3 = indices[j + 3];

            var dx = Vector128.Create(spheres.PosX[j0], spheres.PosX[j1], spheres.PosX[j2], spheres.PosX[j3]) - pxV;
            var dy = Vector128.Create(spheres.PosY[j0], spheres.PosY[j1], spheres.PosY[j2], spheres.PosY[j3]) - pyV;
            var dz = Vector128.Create(spheres.PosZ[j0], spheres.PosZ[j1], spheres.PosZ[j2], spheres.PosZ[j3]) - pzV;
            var distSq = dx * dx + dy * dy + dz * dz;
            var hit = Vector128.LessThan(distSq, minDistV).AsInt32();

            if (hit.GetElement(0) != 0 && TrySeparate(
                    spheres, i, j0, minDistSq, minDiameter, restitution, applyImpulses, awakePairsOnly))
                contacts++;
            if (hit.GetElement(1) != 0 && TrySeparate(
                    spheres, i, j1, minDistSq, minDiameter, restitution, applyImpulses, awakePairsOnly))
                contacts++;
            if (hit.GetElement(2) != 0 && TrySeparate(
                    spheres, i, j2, minDistSq, minDiameter, restitution, applyImpulses, awakePairsOnly))
                contacts++;
            if (hit.GetElement(3) != 0 && TrySeparate(
                    spheres, i, j3, minDistSq, minDiameter, restitution, applyImpulses, awakePairsOnly))
                contacts++;
        }

        for (; j < n; j++)
        {
            pairChecks++;
            if (TrySeparate(
                    spheres,
                    i,
                    indices[j],
                    minDistSq,
                    minDiameter,
                    restitution,
                    applyImpulses,
                    awakePairsOnly))
                contacts++;
        }

        return contacts;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TrySeparate(
        SphereSoA spheres,
        int i,
        int j,
        float minDistSq,
        float minDiameter,
        float restitution,
        bool applyImpulses,
        bool awakePairsOnly)
    {
        if (awakePairsOnly && (spheres.Sleeping[i] || spheres.Sleeping[j]))
            return false;

        var dx = spheres.PosX[j] - spheres.PosX[i];
        var dy = spheres.PosY[j] - spheres.PosY[i];
        var dz = spheres.PosZ[j] - spheres.PosZ[i];
        var distSq = dx * dx + dy * dy + dz * dz;
        if (distSq >= minDistSq)
            return false;

        float overlap;
        if (distSq < 1e-10f)
        {
            dx = 1f;
            dy = 0f;
            dz = 0f;
            overlap = minDiameter;
        }
        else
        {
            var dist = MathF.Sqrt(distSq);
            var invDist = 1f / dist;
            overlap = minDiameter - dist;
            dx *= invDist;
            dy *= invDist;
            dz *= invDist;
        }

        var push = overlap * 0.5f;
        spheres.PosX[i] -= dx * push;
        spheres.PosY[i] -= dy * push;
        spheres.PosZ[i] -= dz * push;
        spheres.PosX[j] += dx * push;
        spheres.PosY[j] += dy * push;
        spheres.PosZ[j] += dz * push;

        if (!applyImpulses || spheres.Sleeping[i] || spheres.Sleeping[j])
            return true;

        var rvx = spheres.VelX[j] - spheres.VelX[i];
        var rvy = spheres.VelY[j] - spheres.VelY[i];
        var rvz = spheres.VelZ[j] - spheres.VelZ[i];
        var vn = rvx * dx + rvy * dy + rvz * dz;
        if (vn >= 0f)
            return true;

        var impulse = -(1f + restitution) * vn * 0.5f;
        spheres.VelX[i] -= dx * impulse;
        spheres.VelY[i] -= dy * impulse;
        spheres.VelZ[i] -= dz * impulse;
        spheres.VelX[j] += dx * impulse;
        spheres.VelY[j] += dy * impulse;
        spheres.VelZ[j] += dz * impulse;
        return true;
    }

    private void BuildGrid(SphereSoA spheres, float cellSize)
    {
        foreach (var list in _cells.Values)
        {
            list.Clear();
            _cellPool.Push(list);
        }

        _cells.Clear();

        var invCell = 1f / cellSize;
        for (var i = 0; i < spheres.Count; i++)
        {
            var key = CellKey(
                (int)MathF.Floor(spheres.PosX[i] * invCell),
                (int)MathF.Floor(spheres.PosZ[i] * invCell));

            if (!_cells.TryGetValue(key, out var list))
            {
                list = _cellPool.Count > 0 ? _cellPool.Pop() : new List<int>(8);
                _cells[key] = list;
            }

            list.Add(i);
        }
    }

    private static int CellKey(int cellX, int cellZ) => cellX * 4096 + cellZ;
}
