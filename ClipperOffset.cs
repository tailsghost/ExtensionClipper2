namespace ExtensionClipper2;

using ExtensionClipper2.Core;
using ExtensionClipper2.Engine;
using ExtensionClipper2.Enums;
using System.Runtime.CompilerServices;
public class ClipperOffsetD
{
    private class Group
    {
        internal PathsD inPaths;
        internal JoinType joinType;
        internal EndType endType;
        internal bool pathsReversed;
        internal int lowestPathIdx;

        public Group(PathsD paths, JoinType joinType, EndType endType = EndType.Polygon)
        {
            this.joinType = joinType;
            this.endType = endType;
            var isClosedPath = endType is EndType.Polygon or EndType.Joined;
            inPaths = new PathsD(paths.Count);

            for (var i = 0; i < paths.Count; i++)
            {
                var path = paths[i];
                inPaths.Add(StripDuplicates(path, isClosedPath));
            }

            if (endType == EndType.Polygon)
            {
                lowestPathIdx = GetLowestPathIdx(inPaths);
                pathsReversed = lowestPathIdx >= 0 && Clipper.Area(inPaths[lowestPathIdx]) < 0.0;
            }
            else
            {
                lowestPathIdx = -1;
                pathsReversed = false;
            }
        }
    }

    public delegate double DeltaCallbackD(PathD path, PathD path_norms, int currPt, int prevPt);


    private readonly List<Group> groupList = new List<Group>();
    private PathD pathOut = new PathD();
    private readonly PathD normals = new PathD();
    private PathsD solution = new PathsD();
    private PolyTreeD? solutionTree;
    private double groupDelta;
    private double delta;
    private double mitLimSqr;
    private double stepsPerRad;
    private double stepSin;
    private double stepCos;
    private JoinType joinType;
    private EndType endType;
    public double ArcTolerance { get; set; }
    public bool MergeGroups { get; set; }
    public double MiterLimit { get; set; }
    public bool PreserveCollinear { get; set; }
    public bool ReverseSolution { get; set; }
    public DeltaCallbackD? DeltaCallback { get; set; }

    public ClipperOffsetD(double miterLimit = 2.0, double arcTolerance = 0.25, bool preserveCollinear = false, bool reverseSolution = false)
    {
        MiterLimit = miterLimit;
        ArcTolerance = arcTolerance;
        MergeGroups = true;
        PreserveCollinear = preserveCollinear;
        ReverseSolution = reverseSolution;
    }

    public void Clear()
    {
        groupList.Clear();
    }

    public void AddPath(PathD path, JoinType joinType, EndType endType)
    {
        if (path.Count != 0)
        {
            var paths = new PathsD(1) { path };
            AddPaths(paths, joinType, endType);
        }
    }

    public void AddPaths(PathsD paths, JoinType joinType, EndType endType)
    {
        if (paths.Count != 0)
        {
            groupList.Add(new Group(paths, joinType, endType));
        }
    }

    private int CalcSolutionCapacity()
    {
        var num = 0;

        for (var i = 0; i < groupList.Count; i++)
        {
            var group = groupList[i];
            num += (group.endType == EndType.Joined ? (group.inPaths.Count * 2) : group.inPaths.Count);
        }
        return num;
    }

    internal bool CheckPathsReversed()
    {
        var result = false;

        for (var i = 0; i < groupList.Count; i++)
        {
            var group = groupList[i];

            if (group.endType == EndType.Polygon)
            {
                result = group.pathsReversed;
                break;
            }
        }
        return result;
    }

    private void ExecuteInternal(double delta)
    {
        if (groupList.Count == 0)
            return;

        EnsureCapacity(solution, CalcSolutionCapacity());
        if (Math.Abs(delta) < 0.05)
        {

            for (var i = 0; i < groupList.Count; i++)
            {
                var group = groupList[i];

                for (var j = 0; j < group.inPaths.Count; j++)
                {
                    solution.Add(group.inPaths[j]);
                }
            }

            return;
        }

        this.delta = delta;
        mitLimSqr = (MiterLimit <= 1.0 ? 2.0 : (2.0 / Clipper.Sqr(MiterLimit)));
        foreach (var group in groupList)
        {
            DoGroupOffset(group);
        }

        if (groupList.Count != 0)
        {
            var flag = CheckPathsReversed();
            var fillRule = (flag ? FillRule.Negative : FillRule.Positive);
            var clipper = new ClipperD();
            clipper.PreserveCollinear = PreserveCollinear;
            clipper.ReverseSolution = ReverseSolution != flag;
            clipper.AddSubject(solution);
            if (solutionTree != null)
            {
                clipper.Execute(ClipType.Union, fillRule, solutionTree);
            }
            else
            {
                clipper.Execute(ClipType.Union, fillRule, solution);
            }
        }
    }

    public void Execute(double delta, PathsD solution)
    {
        solution.Clear();
        this.solution = solution;
        ExecuteInternal(delta);
    }

    public void Execute(double delta, PolyTreeD solutionTree)
    {
        solutionTree.Clear();
        this.solutionTree = solutionTree;
        solution.Clear();
        ExecuteInternal(delta);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static PointD GetUnitNormal(PointD pt1, PointD pt2)
    {
        var dx = pt2.X - pt1.X;
        var dy = pt2.Y - pt1.Y;
        if (Clipper.AlmostEqual(dx, 0.0) && Clipper.AlmostEqual(dy, 0.0))
            return default(PointD);

        var invLen = 1.0 / Math.Sqrt(dx * dx + dy * dy);
        dx *= invLen;
        dy *= invLen;
        return new PointD(dy, -dx);
    }

    public void Execute(DeltaCallbackD deltaCallback, PathsD solution)
    {
        DeltaCallback = deltaCallback;
        Execute(1.0, solution);
    }

    internal static int GetLowestPathIdx(PathsD paths)
    {
        var result = -1;
        var point = new PointD(double.MaxValue, double.MinValue);
        for (var i = 0; i < paths.Count; i++)
        {

            for (var j = 0; j < paths[i].Count; j++)
            {
                var pt = paths[i][j];

                if (Clipper.GreaterThanOrEqual(pt.Y, point.Y) && (!Clipper.GreaterThan(pt.Y, point.Y) || Clipper.LessThan(pt.X, point.X)))
                {
                    result = i;
                    point.X = pt.X;
                    point.Y = pt.Y;
                }
            }
        }
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PointD TranslatePoint(PointD pt, double dx, double dy)
    {
        return new PointD(pt.X + dx, pt.Y + dy);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PointD ReflectPoint(PointD pt, PointD pivot)
    {
        return new PointD(pivot.X + (pivot.X - pt.X), pivot.Y + (pivot.Y - pt.Y));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool AlmostZero(double value, double epsilon = 0.001)
    {
        return Math.Abs(value) < epsilon;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double Hypotenuse(double x, double y)
    {
        return Math.Sqrt(x * x + y * y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PointD NormalizeVector(PointD vec)
    {
        var len = Hypotenuse(vec.X, vec.Y);
        if (AlmostZero(len))
            return new PointD(0.0, 0.0);
        var inv = 1.0 / len;
        return new PointD(vec.X * inv, vec.Y * inv);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PointD GetAvgUnitVector(PointD vec1, PointD vec2)
    {
        return NormalizeVector(new PointD(vec1.X + vec2.X, vec1.Y + vec2.Y));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PointD IntersectPoint(PointD pt1a, PointD pt1b, PointD pt2a, PointD pt2b)
    {
        if (IsAlmostZero(pt1a.X - pt1b.X))
        {
            if (IsAlmostZero(pt2a.X - pt2b.X))
                return new PointD(0.0, 0.0);
            var m = (pt2b.Y - pt2a.Y) / (pt2b.X - pt2a.X);
            var b = pt2a.Y - m * pt2a.X;
            return new PointD(pt1a.X, m * pt1a.X + b);
        }

        if (IsAlmostZero(pt2a.X - pt2b.X))
        {
            var m = (pt1b.Y - pt1a.Y) / (pt1b.X - pt1a.X);
            var b = pt1a.Y - m * pt1a.X;
            return new PointD(pt2a.X, m * pt2a.Y + b);
        }

        var m1 = (pt1b.Y - pt1a.Y) / (pt1b.X - pt1a.X);
        var b1 = pt1a.Y - m1 * pt1a.X;
        var m2 = (pt2b.Y - pt2a.Y) / (pt2b.X - pt2a.X);
        var b2 = pt2a.Y - m2 * pt2a.X;
        if (IsAlmostZero(m1 - m2))
            return new PointD(0.0, 0.0);
        var x = (b2 - b1) / (m1 - m2);
        return new PointD(x, m1 * x + b1);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private PointD GetPerpendic(PointD pt, PointD norm)
    {
        return new PointD(pt.X + norm.X * groupDelta, pt.Y + norm.Y * groupDelta);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private PointD GetPerpendicD(PointD pt, PointD norm)
    {
        return new PointD(pt.X + norm.X * groupDelta, pt.Y + norm.Y * groupDelta);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoBevel(PathD path, int j, int k)
    {
        PointD p1, p2;
        if (j == k)
        {
            var absDelta = Math.Abs(groupDelta);
            p1 = new PointD(path[j].X - absDelta * normals[j].X, path[j].Y - absDelta * normals[j].Y);
            p2 = new PointD(path[j].X + absDelta * normals[j].X, path[j].Y + absDelta * normals[j].Y);
        }
        else
        {
            p1 = new PointD(path[j].X + groupDelta * normals[k].X, path[j].Y + groupDelta * normals[k].Y);
            p2 = new PointD(path[j].X + groupDelta * normals[j].X, path[j].Y + groupDelta * normals[j].Y);
        }
        pathOut.Add(p1);
        pathOut.Add(p2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoSquare(PathD path, int j, int k)
    {
        PointD avg;
        if (j != k)
            avg = GetAvgUnitVector(new PointD(-normals[k].Y, normals[k].X), new PointD(normals[j].Y, -normals[j].X));
        else
            avg = new PointD(normals[j].Y, -normals[j].X);

        var absDelta = Math.Abs(groupDelta);
        var pt = path[j];
        pt = TranslatePoint(pt, absDelta * avg.X, absDelta * avg.Y);
        var pt1 = TranslatePoint(pt, groupDelta * avg.Y, groupDelta * (-avg.X));
        var pt2 = TranslatePoint(pt, groupDelta * (-avg.Y), groupDelta * avg.X);
        var perpendicD = GetPerpendicD(path[k], normals[k]);
        if (j == k)
        {
            var ptInter = IntersectPoint(
                TranslatePoint(perpendicD, avg.X * groupDelta, avg.Y * groupDelta),
                pt1,
                pt2,
                perpendicD);
            pathOut.Add(ReflectPoint(ptInter, pt));
            pathOut.Add(ptInter);
        }
        else
        {
            var perpendicD2 = GetPerpendicD(path[j], normals[k]);
            var ptInter = IntersectPoint(pt1, pt2, perpendicD, perpendicD2);
            pathOut.Add(ptInter);
            pathOut.Add(ReflectPoint(ptInter, pt));
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoMiter(PathD path, int j, int k, double cosA)
    {
        var factor = groupDelta / (cosA + 1.0);
        pathOut.Add(new PointD(path[j].X + (normals[k].X + normals[j].X) * factor,
                               path[j].Y + (normals[k].Y + normals[j].Y) * factor));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoRound(PathD path, int j, int k, double angle)
    {
        if (DeltaCallback != null)
        {
            var absDelta = Math.Abs(groupDelta);
            var arcTol = ArcTolerance;
            var steps = Math.PI / Math.Acos(1.0 - arcTol / absDelta);
            stepSin = Math.Sin(Math.PI * 2.0 / steps);
            stepCos = Math.Cos(Math.PI * 2.0 / steps);
            if (groupDelta < 0.0)
                stepSin = -stepSin;
            stepsPerRad = steps / (Math.PI * 2.0);
        }

        var pt = path[j];
        var vec = new PointD(normals[k].X * groupDelta, normals[k].Y * groupDelta);
        if (j == k)
            vec = new PointD(-vec.X, -vec.Y);
        pathOut.Add(new PointD(pt.X + vec.X, pt.Y + vec.Y));
        var stepsCount = (int)Math.Ceiling(stepsPerRad * Math.Abs(angle));
        for (var i = 1; i < stepsCount; i++)
        {
            vec = new PointD(vec.X * stepCos - stepSin * vec.Y, vec.X * stepSin + vec.Y * stepCos);
            pathOut.Add(new PointD(pt.X + vec.X, pt.Y + vec.Y));
        }
        pathOut.Add(GetPerpendic(pt, normals[j]));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void BuildNormals(PathD path)
    {
        var count = path.Count;
        normals.Clear();
        if (count != 0)
        {
            EnsureCapacity(normals, count);
            for (var i = 0; i < count - 1; i++)
            {
                normals.Add(GetUnitNormal(path[i], path[i + 1]));
            }
            normals.Add(GetUnitNormal(path[count - 1], path[0]));
        }
    }

    private void OffsetPoint(Group group, PathD path, int j, ref int k)
    {
        if (path[j].Equals(path[k]))
        {
            k = j;
            return;
        }

        var cross = CrossProduct(normals[j], normals[k]);
        var dot = DotProduct(normals[j], normals[k]);
        cross = (cross > 1.0 ? 1.0 : (cross < -1.0 ? -1.0 : cross));

        if (DeltaCallback != null)
        {
            groupDelta = DeltaCallback(path, normals, j, k);
            if (group.pathsReversed)
                groupDelta = -groupDelta;
        }

        if (Math.Abs(groupDelta) < Epsilon.GetEpsilonValue())
        {
            pathOut.Add(path[j]);
            return;
        }

        if (dot > -0.999 && cross * groupDelta < 0.0)
        {
            pathOut.Add(GetPerpendic(path[j], normals[k]));
            if (dot < 0.99)
                pathOut.Add(path[j]);
            pathOut.Add(GetPerpendic(path[j], normals[j]));
        }
        else if (dot > 0.999 && joinType != JoinType.Round)
        {
            DoMiter(path, j, k, dot);
        }
        else if (joinType == JoinType.Miter)
        {
            if (dot > mitLimSqr - 1.0)
                DoMiter(path, j, k, dot);
            else
                DoSquare(path, j, k);
        }
        else if (joinType == JoinType.Round)
        {
            DoRound(path, j, k, Math.Atan2(cross, dot));
        }
        else if (joinType == JoinType.Bevel)
        {
            DoBevel(path, j, k);
        }
        else
        {
            DoSquare(path, j, k);
        }
        k = j;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void OffsetPolygon(Group group, PathD path)
    {
        pathOut = new PathD();
        var count = path.Count;
        var k = count - 1;
        for (var i = 0; i < count; i++)
        {
            OffsetPoint(group, path, i, ref k);
        }
        solution.Add(pathOut);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void OffsetOpenJoined(Group group, PathD path)
    {
        OffsetPolygon(group, path);
        path = Clipper.ReversePath(path);
        BuildNormals(path);
        OffsetPolygon(group, path);
    }

    private void OffsetOpenPath(Group group, PathD path)
    {
        pathOut = new PathD();
        var last = path.Count - 1;
        if (DeltaCallback != null)
        {
            groupDelta = DeltaCallback(path, normals, 0, 0);
        }

        if (Math.Abs(groupDelta) < Epsilon.GetEpsilonValue())
        {
            pathOut.Add(path[0]);
        }
        else
        {
            switch (endType)
            {
                case EndType.Butt:
                    DoBevel(path, 0, 0);
                    break;
                case EndType.Round:
                    DoRound(path, 0, 0, Math.PI);
                    break;
                default:
                    DoSquare(path, 0, 0);
                    break;
            }
        }

        var i = 1;
        var k = 0;
        for (; i < last; i++)
        {
            OffsetPoint(group, path, i, ref k);
        }

        for (var iRev = last; iRev > 0; iRev--)
        {
            normals[iRev] = new PointD(-normals[iRev - 1].X, -normals[iRev - 1].Y);
        }
        normals[0] = normals[last];
        if (DeltaCallback != null)
        {
            groupDelta = DeltaCallback(path, normals, last, last);
        }
        if (Math.Abs(groupDelta) < Epsilon.GetEpsilonValue())
        {
            pathOut.Add(path[last]);
        }
        else
        {
            switch (endType)
            {
                case EndType.Butt:
                    DoBevel(path, last, last);
                    break;
                case EndType.Round:
                    DoRound(path, last, last, Math.PI);
                    break;
                default:
                    DoSquare(path, last, last);
                    break;
            }
        }

        var j = last - 1;
        var k2 = last;
        while (j > 0)
        {
            OffsetPoint(group, path, j, ref k2);
            j--;
        }
        solution.Add(pathOut);
    }

    private void DoGroupOffset(Group group)
    {
        if (group.endType == EndType.Polygon)
        {
            if (group.lowestPathIdx < 0)
                delta = Math.Abs(delta);
            groupDelta = (group.pathsReversed ? -delta : delta);
        }
        else
        {
            groupDelta = Math.Abs(delta);
        }

        var absDelta = Math.Abs(groupDelta);
        joinType = group.joinType;
        endType = group.endType;
        if (group.joinType == JoinType.Round || group.endType == EndType.Round)
        {
            var tol = ArcTolerance;
            var steps = Math.PI / Math.Acos(1.0 - tol / absDelta);
            stepSin = Math.Sin(Math.PI * 2.0 / steps);
            stepCos = Math.Cos(Math.PI * 2.0 / steps);
            if (groupDelta < 0.0)
                stepSin = -stepSin;
            stepsPerRad = steps / (Math.PI * 2.0);
        }

        for (var i = 0; i < group.inPaths.Count; i++)
        {
            var inPath = group.inPaths[i];

            pathOut = new PathD();
            switch (inPath.Count)
            {
                case 1:
                {
                    var center = inPath[0];
                    if (DeltaCallback != null)
                    {
                        groupDelta = DeltaCallback(inPath, normals, 0, 0);
                        if (group.pathsReversed)
                            groupDelta = -groupDelta;
                        absDelta = Math.Abs(groupDelta);
                    }
                    if (group.endType == EndType.Round)
                    {
                        var steps = Math.Ceiling(stepsPerRad * 2.0 * Math.PI);
                        pathOut = Clipper.Ellipse(center, absDelta, absDelta, steps);
                    }
                    else
                    {
                        var offset = Math.Ceiling(absDelta);
                        pathOut = new RectD(center.X - offset, center.Y - offset, center.X + offset, center.Y + offset).AsPath();
                    }
                    solution.Add(pathOut);
                    continue;
                }
                case 2:
                    if (group.endType == EndType.Joined)
                    {
                        endType = (group.joinType == JoinType.Round ? EndType.Round : EndType.Square);
                    }
                    break;
            }
            BuildNormals(inPath);
            if (endType == EndType.Polygon)
                OffsetPolygon(group, inPath);
            else if (endType == EndType.Joined)
                OffsetOpenJoined(group, inPath);
            else
                OffsetOpenPath(group, inPath);
        }
    }

    private static double CrossProduct(PointD pt1, PointD pt2)
    {
        return pt1.X * pt2.Y - pt1.Y * pt2.X;
    }

    internal static double DotProduct(PointD pt1, PointD pt2)
    {
        return pt1.X * pt2.X + pt1.Y * pt2.Y;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void EnsureCapacity<T>(List<T> list, int minCapacity)
    {
        if (list.Capacity < minCapacity)
        {
            list.Capacity = minCapacity;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsAlmostZero(double value)
    {
        return Math.Abs(value) <= Epsilon.GetEpsilonValue();
    }

    private static PathD StripDuplicates(PathD path, bool isClosedPath)
    {
        var count = path.Count;
        var path2 = new PathD(count);
        if (count == 0)
        {
            return path2;
        }

        var point = path[0];
        path2.Add(point);
        for (var i = 1; i < count; i++)
        {
            if (!point.Equals(path[i]))
            {
                point = path[i];
                path2.Add(point);
            }
        }

        if (isClosedPath && point.Equals(path2[0]))
        {
            path2.RemoveAt(path2.Count - 1);
        }

        return path2;
    }
}

