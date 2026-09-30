using Novolis.Physics.TestSupport;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Ballistics;
using Novolis.Physics.Gravity;
using Novolis.Physics.Motion;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

file sealed record OrbitAxisRow(string Axis, double ExpectedM, double SimulatedM, double AbsDeltaM);
