using ExtensionClipper2.Core;
using ExtensionClipper2.Engine;
using ExtensionClipper2.Engine.Property;
using System.Runtime.CompilerServices;
using ExtensionClipper2.Enums;
using ClipperD = ExtensionClipper2.Engine.ClipperD;
using ClipType = ExtensionClipper2.Enums.ClipType;
using FillRule = ExtensionClipper2.Enums.FillRule;
using PathsD = ExtensionClipper2.Core.PathsD;
using PathType = ExtensionClipper2.Enums.PathType;
using PolyTreeD = ExtensionClipper2.Engine.PolyTreeD;
using RectD = ExtensionClipper2.Core.RectD;
using ExtensionClipper2.RectClips;

namespace ExtensionClipper2
{

    public static class Epsilon
    {
        private static double _value = 1E-14;
        private static bool _useEpsilon = true;

        internal static double GetEpsilonValue()
            => _value;

        public static void SetEpsilonValue(double value)
        {
            if(value >= 0)
                _value = value;
        }

    }

    public static class Clipper
    {
        private static RectD invalidRectD = new RectD(false);
        public static RectD InvalidRectD => invalidRectD;

        public static bool AlmostEqual(double a, double b)
        {
            //return Math.Abs(a - b) <= Epsilon.GetEpsilonValue() * Math.Max(Math.Abs(a), Math.Abs(b));
           return a == b;
        }
        public static bool VertexValueEquals(Vertex a, Vertex b)
        {
          return  a == b;
            //return AlmostEqual(a.pt.X, b.pt.X) && AlmostEqual(a.pt.Y, b.pt.Y);
        }

        public static bool GreaterThan(double a, double b)
        {
            return a > b;
            return a > b && !AlmostEqual(a, b);
        }

        public static bool LessThan(double a, double b)
        {
            return a < b;
            return a < b && !AlmostEqual(a, b);
        }

        public static bool GreaterThanOrEqual(double a, double b)
        {
            return a >= b;
            return a > b || AlmostEqual(a, b);
        }

        public static bool LessThanOrEqual(double a, double b)
        {
            return a <= b;
            return a < b || AlmostEqual(a, b);
        }

        public static PathsD Intersect(PathsD subject, PathsD clip,
            FillRule fillRule)
        {
            return BooleanOp(ClipType.Intersection,
                subject, clip, fillRule);
        }

        public static PathsD Union(PathsD subject, FillRule fillRule)
        {
            return BooleanOp(ClipType.Union, subject, null, fillRule);
        }

        public static PathsD Union(PathsD subject, PathsD clip,
            FillRule fillRule)
        {
            return BooleanOp(ClipType.Union,
                subject, clip, fillRule);
        }

        public static PathsD Difference(PathsD subject, PathsD clip,
            FillRule fillRule)
        {
            return BooleanOp(ClipType.Difference,
                subject, clip, fillRule);
        }

        public static PathD NewPathD(PathD path, double scale)
        {
            var cnt = path.Count;
            var res = new PathD(cnt);

            for (int i = 0; i < path.Count; i++)
            {
                var pt = path[i];
                res.Add(new PointD(pt));
            }
            return res;
        }

        public static PathsD Xor(PathsD subject, PathsD clip,
            FillRule fillRule)
        {
            return BooleanOp(ClipType.Xor,
                subject, clip, fillRule);
        }

        public static PathsD BooleanOp(ClipType clipType, PathsD subject, PathsD? clip,
            FillRule fillRule)
        {
            var solution = new PathsD();
            var c = new ClipperD();
            c.AddSubject(subject);
            if (clip != null)
                c.AddClip(clip);
            c.Execute(clipType, fillRule, solution);
            return solution;
        }

        public static void BooleanOp(ClipType clipType,
            PathsD? subject, PathsD? clip,
            PolyTreeD polytree, FillRule fillRule)
        {
            if (subject == null) return;
            var c = new ClipperD();
            c.AddPaths(subject, PathType.Subject);
            if (clip != null)
                c.AddPaths(clip, PathType.Clip);
            c.Execute(clipType, fillRule, polytree);
        }

        public static PathsD InflatePaths(PathsD paths, double delta, JoinType joinType,
            EndType endType)
        {
            var co = new ClipperOffsetD(2, 0.01);
            co.AddPaths(paths, joinType, endType);
            var tmp = new PathsD();
            co.Execute(delta, tmp);
            return tmp;
        }

        public static PathsD RectClip(RectD rect, PathsD paths)
        {
            if (rect.IsEmpty() || paths.Count == 0) return new PathsD();
            var rc = new RectClip(rect);
            var tmpPath = rc.Execute(paths);
            return tmpPath;
        }

        public static PathsD RectClip(RectD rect, PathD path)
        {
            if (rect.IsEmpty() || path.Count == 0) return new();
            var tmp = new PathsD { path };
            return RectClip(rect, tmp);
        }

        public static PathsD RectClipLines(RectD rect,
            PathsD paths)
        {
            if (rect.IsEmpty() || paths.Count == 0) return new();
            var rc = new RectClipLines(rect);
            paths = rc.Execute(paths);
            return paths;
        }

        public static PathsD RectClipLines(RectD rect, PathD path)
        {
            if (rect.IsEmpty() || path.Count == 0) return new();
            var tmp = new PathsD { path };
            return RectClipLines(rect, tmp);
        }

        public static double Area(PathD path)
        {
            var a = 0.0;
            var cnt = path.Count;
            if (cnt < 3) return 0.0;
            var prevPt = path[cnt - 1];

            for (var i = 0; i < path.Count; i++)
            {
                var pt = path[i];
                a += (prevPt.Y + pt.Y) * (prevPt.X - pt.X);
                prevPt = pt;
            }
            return a * 0.5;
        }

        public static double Area(PathsD paths)
        {
            var a = 0.0;
            for (var i = 0; i < paths.Count; i++)
            {
                var p = paths[i];
                a += Area(p);
            }
            return a;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsPositive(PathD poly)
        {
            return GreaterThanOrEqual(Area(poly), 0);
        }

        public static string PathDToString(PathD path)
        {
            var result = "";

            for (var i = 0; i < path.Count; i++)
            {
                var pt = path[i];
                result += pt.ToString();
            }
            return result + '\n';
        }
        public static string PathsDToString(PathsD paths)
        {
            var result = "";
            for (var i = 0; i < paths.Count; i++)
            {
                var path = paths[i];
                result += PathDToString(path);
            }
            return result;
        }

        public static PathD TranslatePath(PathD path, double dx, double dy)
        {
            var result = new PathD(path.Count);

            for (var i = 0; i < path.Count; i++)
            {
                var pt = path[i];
                result.Add(new PointD(pt.X + dx, pt.Y + dy));
            }

            return result;
        }

        public static PathsD TranslatePaths(PathsD paths, double dx, double dy)
        {
            var result = new PathsD(paths.Count);

            for (var i = 0; i < paths.Count; i++)
            {
                var p = paths[i];
                result.Add(TranslatePath(p, dx, dy));
            }

            return result;
        }

        public static PathD ReversePath(PathD path)
        {
            var result = new PathD(path);
            result.Reverse();
            return result;
        }

        public static PathsD ReversePaths(PathsD paths)
        {
            var result = new PathsD(paths.Count);

            for (var i = 0; i < paths.Count; i++)
            {
                var path = paths[i];
                result.Add(ReversePath(path));
            }

            return result;
        }

        public static RectD GetBounds(PathD path)
        {
            var result = InvalidRectD;

            for (var i = 0; i < path.Count; i++)
            {
                var pt = path[i];
                if (LessThan(pt.X, result.Left)) result.Left = pt.X;
                if (GreaterThan(pt.X, result.Right)) result.Right = pt.X;
                if (LessThan(pt.Y, result.Top)) result.Top = pt.Y;
                if (GreaterThan(pt.Y, result.Bottom)) result.Bottom = pt.Y;
            }

            return Math.Abs(result.Left - double.MaxValue) < Epsilon.GetEpsilonValue() ? new RectD() : result;
        }

        public static RectD GetBounds(PathsD paths)
        {
            var result = InvalidRectD;

            for (var i = 0; i < paths.Count; i++)
            {
                var path = paths[i];

                for (var j = 0; j < path.Count; j++)
                {
                    var pt = path[j];
                    if (LessThan(pt.X, result.Left)) result.Left = pt.X;
                    if (GreaterThan(pt.X, result.Right)) result.Right = pt.X;
                    if (LessThan(pt.Y, result.Top)) result.Top = pt.Y;
                    if (GreaterThan(pt.Y, result.Bottom)) result.Bottom = pt.Y;

                }
            }

            return Math.Abs(result.Left - double.MaxValue) < Epsilon.GetEpsilonValue() ? new RectD() : result;
        }

        public static PathD MakePath(double[] arr)
        {
            var len = arr.Length / 2;
            var p = new PathD(len);
            for (var i = 0; i < len; i++)
                p.Add(new PointD(arr[i * 2], arr[i * 2 + 1]));
            return p;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Sqr(double val)
        {
            return val * val;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static PointD MidPoint(PointD pt1, PointD pt2)
        {
            return new PointD((pt1.X + pt2.X) / 2, (pt1.Y + pt2.Y) / 2);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InflateRect(ref RectD rec, double dx, double dy)
        {
            rec.Left -= dx;
            rec.Right += dx;
            rec.Top -= dy;
            rec.Bottom += dy;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool PointsNearEqual(PointD pt1, PointD pt2, double distanceSqrd)
        {
            return LessThan(Sqr(pt1.X - pt2.X) + Sqr(pt1.Y - pt2.Y), distanceSqrd);
        }

        public static PathD StripNearDuplicates(PathD path,
            double minEdgeLenSqrd, bool isClosedPath)
        {
            var cnt = path.Count;
            var result = new PathD(cnt);
            if (cnt == 0) return result;
            var lastPt = path[0];
            result.Add(lastPt);
            for (var i = 1; i < cnt; i++)
                if (!PointsNearEqual(lastPt, path[i], minEdgeLenSqrd))
                {
                    lastPt = path[i];
                    result.Add(lastPt);
                }

            if (isClosedPath && PointsNearEqual(lastPt, result[0], minEdgeLenSqrd))
            {
                result.RemoveAt(result.Count - 1);
            }

            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void AddPolyNodeToPathsD(PolyPathD polyPath, PathsD paths)
        {
            if (polyPath.Polygon!.Count > 0)
                paths.Add(polyPath.Polygon);
            for (var i = 0; i < polyPath.Count; i++)
                AddPolyNodeToPathsD((PolyPathD)polyPath._childs[i], paths);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static PathsD PolyTreeToPathsD(PolyTreeD polyTree)
        {
            var result = new PathsD();

            for (var i = 0; i < polyTree.Count; i++)
            {
                var p = polyTree[i];
            }

            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double PerpendicDistFromLineSqrd(PointD pt, PointD line1, PointD line2)
        {
            var a = pt.X - line1.X;
            var b = pt.Y - line1.Y;
            var c = line2.X - line1.X;
            var d = line2.Y - line1.Y;
            if (AlmostEqual(c,0) && AlmostEqual(d, 0)) return 0;
            return Sqr(a * d - c * b) / (c * c + d * d);
        }

        internal static void RDP(PathD path, int begin, int end, double epsSqrd, List<bool> flags)
        {
            while (true)
            {
                var idx = 0;
                var max_d = 0.0;
                while (end > begin && path[begin] == path[end]) flags[end--] = false;
                for (var i = begin + 1; i < end; ++i)
                {
                    var d = PerpendicDistFromLineSqrd(path[i], path[begin], path[end]);
                    if (LessThan(d, max_d)) continue;
                    max_d = d;
                    idx = i;
                }

                if (LessThan(max_d, epsSqrd)) return;
                flags[idx] = true;
                if (idx > begin + 1) RDP(path, begin, idx, epsSqrd, flags);
                if (idx < end - 1)
                {
                    begin = idx;
                    continue;
                }

                break;
            }
        }

        public static PathD RamerDouglasPeucker(PathD path, double epsilon)
        {
            var len = path.Count;
            if (len < 5) return path;
            var flags = new List<bool>(new bool[len]) { [0] = true, [len - 1] = true };
            RDP(path, 0, len - 1, Sqr(epsilon), flags);
            var result = new PathD(len);
            for (var i = 0; i < len; ++i)
                if (flags[i]) result.Add(path[i]);
            return result;
        }

        public static PathsD RamerDouglasPeucker(PathsD paths, double epsilon)
        {
            var result = new PathsD(paths.Count);

            for (var i = 0; i < paths.Count; i++)
            {
                var path = paths[i];
                result.Add(RamerDouglasPeucker(path, epsilon));
            }

            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int GetNext(int current, int high, ref bool[] flags)
        {
            ++current;
            while (current <= high && flags[current]) ++current;
            if (current <= high) return current;
            current = 0;
            while (flags[current]) ++current;
            return current;
        }

        private static int GetPrior(int current, int high, ref bool[] flags)
        {
            if (current == 0) current = high;
            else --current;
            while (current > 0 && flags[current]) --current;
            if (!flags[current]) return current;
            current = high;
            while (flags[current]) --current;
            return current;
        }

        public static PathsD SimplifyPaths(PathsD paths,
            double epsilon, bool isClosedPath = true)
        {
            var result = new PathsD(paths.Count);

            for (var i = 0; i < paths.Count; i++)
            {
                var path = paths[i];
                result.Add(SimplifyPath(path, epsilon, isClosedPath));
            }

            return result;
        }

        public static PathD SimplifyPath(PathD path,
            double epsilon, bool isClosedPath = true)
        {
            int len = path.Count, high = len - 1;
            var epsSqr = Sqr(epsilon);
            if (len < 4) return path;

            var flags = new bool[len];
            var dsq = new double[len];
            var curr = 0;

            if (isClosedPath)
            {
                dsq[0] = PerpendicDistFromLineSqrd(path[0], path[high], path[1]);
                dsq[high] = PerpendicDistFromLineSqrd(path[high], path[0], path[high - 1]);
            }
            else
            {
                dsq[0] = double.MaxValue;
                dsq[high] = double.MaxValue;
            }

            for (var i = 1; i < high; ++i)
                dsq[i] = PerpendicDistFromLineSqrd(path[i], path[i - 1], path[i + 1]);

            for (; ; )
            {
                if (dsq[curr] > epsSqr)
                {
                    var start = curr;
                    do
                    {
                        curr = GetNext(curr, high, ref flags);
                    } while (curr != start && dsq[curr] > epsSqr);
                    if (curr == start) break;
                }

                var prev = GetPrior(curr, high, ref flags);
                var next = GetNext(curr, high, ref flags);
                if (next == prev) break;

                int prior2;
                if (dsq[next] < dsq[curr])
                {
                    prior2 = prev;
                    prev = curr;
                    curr = next;
                    next = GetNext(next, high, ref flags);
                }
                else
                    prior2 = GetPrior(prev, high, ref flags);

                flags[curr] = true;
                curr = next;
                next = GetNext(next, high, ref flags);
                if (isClosedPath || ((curr != high) && (curr != 0)))
                    dsq[curr] = PerpendicDistFromLineSqrd(path[curr], path[prev], path[next]);
                if (isClosedPath || ((prev != 0) && (prev != high)))
                    dsq[prev] = PerpendicDistFromLineSqrd(path[prev], path[prior2], path[curr]);
            }
            var result = new PathD(len);
            for (var i = 0; i < len; i++)
                if (!flags[i]) result.Add(path[i]);
            return result;
        }


        public static PathD Ellipse(PointD center,
            double radiusX, double radiusY = 0, int steps = 0)
        {
            if (radiusX <= 0) return new();
            if (radiusY <= 0) radiusY = radiusX;
            if (steps <= 2)
                steps = (int)Math.Ceiling(Math.PI * Math.Sqrt((radiusX + radiusY) / 2));

            var si = Math.Sin(2 * Math.PI / steps);
            var co = Math.Cos(2 * Math.PI / steps);
            double dx = co, dy = si;
            var result = new PathD(steps) { new PointD(center.X + radiusX, center.Y) };
            for (var i = 1; i < steps; ++i)
            {
                result.Add(new PointD(center.X + radiusX * dx, center.Y + radiusY * dy));
                var x = dx * co - dy * si;
                dy = dy * co + dx * si;
                dx = x;
            }
            return result;
        }

        private static void ShowPolyPathStructure(PolyPathD pp, int level)
        {
            var spaces = new string(' ', level * 2);
            var caption = (pp.IsHole ? "Hole " : "Outer ");
            if (pp.Count == 0)
            {
                Console.WriteLine(spaces + caption);
            }
            else
            {
                Console.WriteLine(spaces + caption + $"({pp.Count})");
                foreach (PolyPathD child in pp) { ShowPolyPathStructure(child, level + 1); }
            }
        }

        public static void ShowPolyTreeStructure(PolyTreeD polytree)
        {
            Console.WriteLine("Polytree Root");
            foreach (PolyPathD child in polytree) { ShowPolyPathStructure(child, 1); }
        }


    }
}
