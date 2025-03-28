using System.IO;
using System.Runtime.CompilerServices;
using ExtensionClipper2.Enums;

namespace ExtensionClipper2
{
    internal static class Epsilon
    {
        private const double _value = 1E-10;

        internal static double GetEpsilonValue()
            => _value;
    }
}

namespace ExtensionClipper2.Core
{
    public static class InternalClipper
    {
        internal const double MaxValue = double.MaxValue;
        internal const double MaxCoord = MaxValue / 4;
        internal const double MinCoord = double.MinValue / 4;
        internal const double InvalidValue = double.MaxValue;

        internal const double floatingPointTolerance = 1E-12;

        private static readonly string precision_range_error = "Error: Precision is out of range.";

        public static double CrossProduct(PointD pt1, PointD pt2, PointD pt3)
        {
            return (pt2.X - pt1.X) * (pt3.Y - pt2.Y) -
                   (pt2.Y - pt1.Y) * (pt3.X - pt2.X);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void CheckPrecision(int precision)
        {
            if (precision is < -8 or > 8)
                throw new Exception(precision_range_error);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool IsAlmostZero(double value)
        {
            // Используем новую константу Epsilon для сравнения с нулём
            return (Math.Abs(value) <= Epsilon.GetEpsilonValue());
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static int TriSign(double x)
        {
            if (x < 0) return -1;
            return x > 1 ? 1 : 0;
        }

        internal static bool ProductsAreEqual(double a, double b, double c, double d)
        {
            var absA = Math.Abs(a);
            var absB = Math.Abs(b);
            var absC = Math.Abs(c);
            var absD = Math.Abs(d);

            var sign_ab = TriSign(a) * TriSign(b);
            var sign_cd = TriSign(c) * TriSign(d);

            return sign_ab == sign_cd;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool IsCollinear(PointD pt1, PointD sharedPt, PointD pt2)
        {
            var a = sharedPt.X - pt1.X;
            var b = pt2.Y - sharedPt.Y;
            var c = sharedPt.Y - pt1.Y;
            var d = pt2.X - sharedPt.X;
            return ProductsAreEqual(a, b, c, d);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static double DotProduct(PointD pt1, PointD pt2, PointD pt3)
        {
            return ((pt2.X - pt1.X) * (pt3.X - pt2.X) +
                    (pt2.Y - pt1.Y) * (pt3.Y - pt2.Y));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static double CrossProduct(PointD vec1, PointD vec2)
        {
            return (vec1.Y * vec2.X - vec2.Y * vec1.X);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static double DotProduct(PointD vec1, PointD vec2)
        {
            return (vec1.X * vec2.X + vec1.Y * vec2.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static double CheckCast(double val)
        {
            return val is >= MaxCoord or <= MinCoord ? InvalidValue : val;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool GetSegmentIntersectPt(PointD ln1a,
            PointD ln1b, PointD ln2a, PointD ln2b, out PointD ip)
        {
            var dy1 = (ln1b.Y - ln1a.Y);
            var dx1 = (ln1b.X - ln1a.X);
            var dy2 = (ln2b.Y - ln2a.Y);
            var dx2 = (ln2b.X - ln2a.X);
            var det = dy1 * dx2 - dy2 * dx1;
            if (Math.Abs(det) < Epsilon.GetEpsilonValue())
            {
                ip = new PointD();
                return false;
            }

            var t = ((ln1a.X - ln2a.X) * dy2 - (ln1a.Y - ln2a.Y) * dx2) / det;
            if (t <= 0.0)
                ip = ln1a;
            else if (t >= 1.0)
                ip = ln1b;
            else
            {
                ip.X = ln1a.X + t * dx1;
                ip.Y = ln1a.Y + t * dy1;
            }

            return true;
        }

        internal static bool SegsIntersect(PointD seg1a,
            PointD seg1b, PointD seg2a, PointD seg2b, bool inclusive = false)
        {
            if (!inclusive)
                return (CrossProduct(seg1a, seg2a, seg2b) *
                        CrossProduct(seg1b, seg2a, seg2b) < 0) &&
                       (CrossProduct(seg2a, seg1a, seg1b) *
                        CrossProduct(seg2b, seg1a, seg1b) < 0);
            var res1 = CrossProduct(seg1a, seg2a, seg2b);
            var res2 = CrossProduct(seg1b, seg2a, seg2b);
            if (res1 * res2 > 0) return false;
            var res3 = CrossProduct(seg2a, seg1a, seg1b);
            var res4 = CrossProduct(seg2b, seg1a, seg1b);
            if (res3 * res4 > 0) return false;
            return (res1 != 0 || res2 != 0 || res3 != 0 || res4 != 0);
        }

        public static PointD GetClosestPtOnSegment(PointD offPt,
            PointD seg1, PointD seg2)
        {
            if (seg1.X == seg2.X && seg1.Y == seg2.Y) return seg1;
            var dx = (seg2.X - seg1.X);
            var dy = (seg2.Y - seg1.Y);
            var q = ((offPt.X - seg1.X) * dx +
                     (offPt.Y - seg1.Y) * dy) / ((dx * dx) + (dy * dy));
            if (q < 0) q = 0;
            else if (q > 1) q = 1;
            return new PointD(
                seg1.X + q * dx,
                seg1.Y + q * dy
            );
        }

        public static PointInPolygonResult PointInPolygon(PointD pt, PathD polygon)
        {
            int len = polygon.Count, start = 0;
            if (len < 3) return PointInPolygonResult.IsOutside;

            while (start < len && IsAlmostZero(polygon[start].Y - pt.Y))
                start++;
            if (start == len) return PointInPolygonResult.IsOutside;

            double d;
            bool isAbove = polygon[start].Y < pt.Y, startingAbove = isAbove;
            int val = 0, i = start + 1, end = len;
            while (true)
            {
                if (i == end)
                {
                    if (end == 0 || start == 0) break;
                    end = start;
                    i = 0;
                }

                if (isAbove)
                {
                    while (i < end && polygon[i].Y < pt.Y) i++;
                }
                else
                {
                    while (i < end && polygon[i].Y > pt.Y) i++;
                }

                if (i == end) continue;

                PointD curr = polygon[i], prev;
                if (i > 0) prev = polygon[i - 1];
                else prev = polygon[len - 1];

                if (IsAlmostZero(curr.Y - pt.Y))
                {
                    if (IsAlmostZero(curr.X - pt.X) || (IsAlmostZero(curr.Y - prev.Y) &&
                        ((pt.X < prev.X) != (pt.X < curr.X))))
                        return PointInPolygonResult.IsOn;
                    i++;
                    if (i == start) break;
                    continue;
                }


                if (pt.X > prev.X && pt.X > curr.X)
                {
                    val = 1 - val;
                }
                else
                {
                    d = CrossProduct(prev, curr, pt);
                    if (IsAlmostZero(d)) return PointInPolygonResult.IsOn;
                    if ((d < 0) == isAbove) val = 1 - val;
                }
                isAbove = !isAbove;
                i++;
            }

            if (isAbove == startingAbove)
                return val == 0 ? PointInPolygonResult.IsOutside : PointInPolygonResult.IsInside;
            if (i == len) i = 0;
            d = i == 0 ? CrossProduct(polygon[len - 1], polygon[0], pt) : CrossProduct(polygon[i - 1], polygon[i], pt);
            if (IsAlmostZero(d)) return PointInPolygonResult.IsOn;
            if ((d < 0) == isAbove) val = 1 - val;

            return val == 0 ? PointInPolygonResult.IsOutside : PointInPolygonResult.IsInside;
        }
    }
}