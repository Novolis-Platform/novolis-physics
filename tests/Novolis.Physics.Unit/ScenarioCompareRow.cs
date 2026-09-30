using Novolis.Physics.TestSupport;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Ballistics;
using Novolis.Physics.Gravity;
using Novolis.Physics.Motion;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

file sealed record ScenarioCompareRow(string Metric, double Expected, double Simulated, double AbsError);
