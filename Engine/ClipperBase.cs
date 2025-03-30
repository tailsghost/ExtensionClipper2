using ExtensionClipper2.Engine.Property;
using ExtensionClipper2.Enums;
using System.Runtime.CompilerServices;
using ExtensionClipper2.Core;

namespace ExtensionClipper2.Engine;

public class ClipperBase
{
    private ClipType _cliptype;
    private FillRule _fillrule;
    private Active? _actives;
    private Active? _sel;
    private readonly List<LocalMinima> _minimaList;
    private readonly List<IntersectNode> _intersectList;
    private readonly List<Vertex> _vertexList;
    private readonly List<OutRec> _outrecList;
    private readonly List<double> _scanlineList;
    private readonly List<HorzSegment> _horzSegList;
    private readonly List<HorzJoin> _horzJoinList;
    private int _currentLocMin;
    private double _currentBotY;
    private bool _isSortedMinimaList;
    private bool _hasOpenPaths;
    internal bool _using_polytree;
    internal bool _succeeded;
    public bool PreserveCollinear { get; set; }
    public bool ReverseSolution { get; set; }

    public ClipperBase()
    {
        _minimaList = new List<LocalMinima>();
        _intersectList = new List<IntersectNode>();
        _vertexList = new List<Vertex>();
        _outrecList = new List<OutRec>();
        _scanlineList = new List<double>();
        _horzSegList = new List<HorzSegment>();
        _horzJoinList = new List<HorzJoin>();
        PreserveCollinear = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsOdd(int val)
    {
        return ((val & 1) != 0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsHotEdge(Active ae)
    {
        return ae.outrec != null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsOpen(Active ae)
    {
        return ae.localMin.isOpen;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsOpenEnd(Active ae)
    {
        return ae.localMin.isOpen && IsOpenEnd(ae.vertexTop!);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsOpenEnd(Vertex v)
    {
        return (v.flags & (VertexFlags.OpenStart | VertexFlags.OpenEnd)) != VertexFlags.None;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Active? GetPrevHotEdge(Active ae)
    {
        Active? prev = ae.prevInAEL;
        while (prev != null && (IsOpen(prev) || !IsHotEdge(prev)))
            prev = prev.prevInAEL;
        return prev;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsFront(Active ae)
    {
        return (ae == ae.outrec!.frontEdge);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double GetDx(PointD pt1, PointD pt2)
    {
        var dy = pt2.Y - pt1.Y;
        if (Math.Abs(dy) < Epsilon.GetEpsilonValue())
        {
            dy = 0;
        }
        if (dy != 0)
            return (pt2.X - pt1.X) / dy;
        return Clipper.GreaterThan(pt2.X, pt1.X) ? double.NegativeInfinity : double.PositiveInfinity;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double TopX(Active ae, double currentY)
    {
        if (Clipper.AlmostEqual(currentY, ae.top.Y) || Clipper.AlmostEqual(ae.top.X, ae.bot.X)) return ae.top.X;

        if (Clipper.AlmostEqual(currentY, ae.bot.Y)) return ae.bot.X;

        return ae.bot.X + ae.dx * (currentY - ae.bot.Y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsHorizontal(Active ae)
    {
        return Clipper.AlmostEqual(ae.top.Y, ae.bot.Y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsHeadingRightHorz(Active ae)
    {
        return (double.IsNegativeInfinity(ae.dx));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsHeadingLeftHorz(Active ae)
    {
        return (double.IsPositiveInfinity(ae.dx));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SwapActives(ref Active ae1, ref Active ae2)
    {
        (ae2, ae1) = (ae1, ae2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PathType GetPolyType(Active ae)
    {
        return ae.localMin.polytype;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsSamePolyType(Active ae1, Active ae2)
    {
        return ae1.localMin.polytype == ae2.localMin.polytype;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetDx(Active ae)
    {
        ae.dx = GetDx(ae.bot, ae.top);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex NextVertex(Active ae)
    {
        return ae.windDx > 0 ? ae.vertexTop!.next! : ae.vertexTop!.prev!;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex PrevPrevVertex(Active ae)
    {
        return ae.windDx > 0 ? ae.vertexTop!.prev!.prev! : ae.vertexTop!.next!.next!;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMaxima(Vertex vertex)
    {
        return ((vertex.flags & VertexFlags.LocalMax) != VertexFlags.None);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMaxima(Active ae)
    {
        return IsMaxima(ae.vertexTop!);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Active? GetMaximaPair(Active ae)
    {
        var ae2 = ae.nextInAEL;
        while (ae2 != null)
        {
            if (ae2.vertexTop == ae.vertexTop) return ae2;
            ae2 = ae2.nextInAEL;
        }
        return null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex? GetCurrYMaximaVertex_Open(Active ae)
    {
        var result = ae.vertexTop;
        if (ae.windDx > 0)
            while (Clipper.AlmostEqual(result!.next!.pt.Y, result.pt.Y) &&
                   ((result.flags & (VertexFlags.OpenEnd |
                                     VertexFlags.LocalMax)) == VertexFlags.None))
                result = result.next;
        else
            while (Clipper.AlmostEqual(result!.prev!.pt.Y, result.pt.Y) &&
                   ((result.flags & (VertexFlags.OpenEnd |
                                     VertexFlags.LocalMax)) == VertexFlags.None))
                result = result.prev;
        if (!IsMaxima(result)) result = null;
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex? GetCurrYMaximaVertex(Active ae)
    {
        var result = ae.vertexTop;
        if (ae.windDx > 0)
            while (Clipper.AlmostEqual(result!.next!.pt.Y, result.pt.Y)) result = result.next;
        else
            while (Clipper.AlmostEqual(result!.prev!.pt.Y, result.pt.Y)) result = result.prev;

        if (!IsMaxima(result)) result = null;
        return result;
    }

    private struct IntersectListSort : IComparer<IntersectNode>
    {
        public readonly int Compare(IntersectNode a, IntersectNode b)
        {
            if (!Clipper.AlmostEqual(a.pt.Y, b.pt.Y)) return (Clipper.GreaterThan(a.pt.Y, b.pt.Y)) ? -1 : 1;
            var result = Clipper.AlmostEqual(a.pt.X, b.pt.X);
            if (result) return 0;
            return (!result && Clipper.LessThan(a.pt.X, b.pt.X)) ? -1 : 1;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetSides(OutRec outrec, Active startEdge, Active endEdge)
    {
        outrec.frontEdge = startEdge;
        outrec.backEdge = endEdge;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SwapOutrecs(Active ae1, Active ae2)
    {
        var or1 = ae1.outrec;
        var or2 = ae2.outrec;
        if (or1 == or2)
        {
            var ae = or1!.frontEdge;
            or1.frontEdge = or1.backEdge;
            or1.backEdge = ae;
            return;
        }

        if (or1 != null)
        {
            if (ae1 == or1.frontEdge)
                or1.frontEdge = ae2;
            else
                or1.backEdge = ae2;
        }

        if (or2 != null)
        {
            if (ae2 == or2.frontEdge)
                or2.frontEdge = ae1;
            else
                or2.backEdge = ae1;
        }

        ae1.outrec = or2;
        ae2.outrec = or1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetOwner(OutRec outrec, OutRec newOwner)
    {
        while (newOwner.owner != null && newOwner.owner.pts == null)
            newOwner.owner = newOwner.owner.owner;

        var tmp = newOwner;
        while (tmp != null && tmp != outrec)
            tmp = tmp.owner;
        if (tmp != null)
            newOwner.owner = outrec.owner;
        outrec.owner = newOwner;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double Area(OutPt op)
    {
        var area = 0.0;
        var op2 = op;
        do
        {
            area += op2.prev.pt.Y + op2.pt.Y * op2.prev.pt.X - op2.pt.X;
            op2 = op2.next!;
        } while (op2 != op);
        return area * 0.5;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double AreaTriangle(PointD pt1, PointD pt2, PointD pt3)
    {
        return (pt3.Y + pt1.Y) * (pt3.X - pt1.X) +
            (pt1.Y + pt2.Y) * (pt1.X - pt2.X) +
            (pt2.Y + pt3.Y) * (pt2.X - pt3.X);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutRec? GetRealOutRec(OutRec? outRec)
    {
        while (outRec is { pts: null })
            outRec = outRec.owner;
        return outRec;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidOwner(OutRec? outRec, OutRec? testOwner)
    {
        while ((testOwner != null) && (testOwner != outRec))
            testOwner = testOwner.owner;
        return testOwner == null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void UncoupleOutRec(Active ae)
    {
        var outrec = ae.outrec;
        if (outrec == null) return;
        outrec.frontEdge!.outrec = null;
        outrec.backEdge!.outrec = null;
        outrec.frontEdge = null;
        outrec.backEdge = null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OutrecIsAscending(Active hotEdge)
    {
        return (hotEdge == hotEdge.outrec!.frontEdge);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SwapFrontBackSides(OutRec outrec)
    {
        Active ae2 = outrec.frontEdge!;
        outrec.frontEdge = outrec.backEdge;
        outrec.backEdge = ae2;
        outrec.pts = outrec.pts!.next;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool EdgesAdjacentInAEL(IntersectNode inode)
    {
        return (inode.edge1.nextInAEL == inode.edge2) || (inode.edge1.prevInAEL == inode.edge2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void ClearSolutionOnly()
    {
        while (_actives != null) DeleteFromAEL(_actives);
        _scanlineList.Clear();
        DisposeIntersectNodes();
        _outrecList.Clear();
        _horzSegList.Clear();
        _horzJoinList.Clear();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Clear()
    {
        ClearSolutionOnly();
        _minimaList.Clear();
        _vertexList.Clear();
        _currentLocMin = 0;
        _isSortedMinimaList = false;
        _hasOpenPaths = false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void Reset()
    {
        if (!_isSortedMinimaList)
        {
            _minimaList.Sort(new LocMinSorter());
            _isSortedMinimaList = true;
        }

        _scanlineList.EnsureCapacity(_minimaList.Count);
        for (var i = _minimaList.Count - 1; i >= 0; i--)
            _scanlineList.Add(_minimaList[i].vertex.pt.Y);

        _currentBotY = 0;
        _currentLocMin = 0;
        _actives = null;
        _sel = null;
        _succeeded = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void InsertScanline(double y)
    {
        var index = _scanlineList.BinarySearch(y);
        if (index >= 0) return;
        index = ~index;
        _scanlineList.Insert(index, y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool PopScanline(out double y)
    {
        var cnt = _scanlineList.Count - 1;
        if (cnt < 0)
        {
            y = 0;
            return false;
        }

        y = _scanlineList[cnt];
        _scanlineList.RemoveAt(cnt--);
        while (cnt >= 0 && y == _scanlineList[cnt])
            _scanlineList.RemoveAt(cnt--);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool HasLocMinAtY(double y)
    {
        return (_currentLocMin < _minimaList.Count && Clipper.AlmostEqual(_minimaList[_currentLocMin].vertex.pt.Y, y));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private LocalMinima PopLocalMinima()
    {
        return _minimaList[_currentLocMin++];
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddSubject(PathD path)
    {
        AddPath(path, PathType.Subject);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddOpenSubject(PathD path)
    {
        AddPath(path, PathType.Subject, true);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddClip(PathD path)
    {
        AddPath(path, PathType.Clip);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void AddPath(PathD path, PathType polytype, bool isOpen = false)
    {
        var tmp = new PathsD(1) { path };
        AddPaths(tmp, polytype, isOpen);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void AddPaths(PathsD paths, PathType polytype, bool isOpen = false)
    {
        if (isOpen) _hasOpenPaths = true;
        _isSortedMinimaList = false;
        ClipperEngine.AddPathsToVertexList(paths, polytype, isOpen, _minimaList, _vertexList);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void AddReuseableData(ReuseableDataContainer reuseableData)
    {
        if (reuseableData._minimaList.Count == 0) return;
        _isSortedMinimaList = false;
        foreach (LocalMinima lm in reuseableData._minimaList)
        {
            _minimaList.Add(new LocalMinima(lm.vertex, lm.polytype, lm.isOpen));
            if (lm.isOpen) _hasOpenPaths = true;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsContributingClosed(Active ae)
    {
        switch (_fillrule)
        {
            case FillRule.Positive:
                if (ae.windCount != 1) return false;
                break;
            case FillRule.Negative:
                if (ae.windCount != -1) return false;
                break;
            case FillRule.NonZero:
                if (Math.Abs(ae.windCount) != 1) return false;
                break;
        }

        switch (_cliptype)
        {
            case ClipType.Intersection:
                return _fillrule switch
                {
                    FillRule.Positive => ae.windCount2 > 0,
                    FillRule.Negative => ae.windCount2 < 0,
                    _ => ae.windCount2 != 0
                };

            case ClipType.Union:
                return _fillrule switch
                {
                    FillRule.Positive => ae.windCount2 <= 0,
                    FillRule.Negative => ae.windCount2 >= 0,
                    _ => ae.windCount2 == 0
                };

            case ClipType.Difference:
                var result = _fillrule switch
                {
                    FillRule.Positive => (ae.windCount2 <= 0),
                    FillRule.Negative => (ae.windCount2 >= 0),
                    _ => (ae.windCount2 == 0)
                };
                return (GetPolyType(ae) == PathType.Subject) ? result : !result;

            case ClipType.Xor:
                return true;

            default:
                return false;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsContributingOpen(Active ae)
    {
        bool isInClip, isInSubj;
        switch (_fillrule)
        {
            case FillRule.Positive:
                isInSubj = ae.windCount > 0;
                isInClip = ae.windCount2 > 0;
                break;
            case FillRule.Negative:
                isInSubj = ae.windCount < 0;
                isInClip = ae.windCount2 < 0;
                break;
            default:
                isInSubj = ae.windCount != 0;
                isInClip = ae.windCount2 != 0;
                break;
        }

        var result = _cliptype switch
        {
            ClipType.Intersection => isInClip,
            ClipType.Union => !isInSubj && !isInClip,
            _ => !isInClip
        };
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetWindCountForClosedPathEdge(Active ae)
    {
        var ae2 = ae.prevInAEL;
        PathType pt = GetPolyType(ae);
        while (ae2 != null && (GetPolyType(ae2) != pt || IsOpen(ae2))) ae2 = ae2.prevInAEL;

        if (ae2 == null)
        {
            ae.windCount = ae.windDx;
            ae2 = _actives;
        }
        else if (_fillrule == FillRule.EvenOdd)
        {
            ae.windCount = ae.windDx;
            ae.windCount2 = ae2.windCount2;
            ae2 = ae2.nextInAEL;
        }
        else
        {
            if (ae2.windCount * ae2.windDx < 0)
            {
                if (Math.Abs(ae2.windCount) > 1)
                {
                    if (ae2.windDx * ae.windDx < 0)
                        ae.windCount = ae2.windCount;
                    else
                        ae.windCount = ae2.windCount + ae.windDx;
                }
                else
                    ae.windCount = (IsOpen(ae) ? 1 : ae.windDx);
            }
            else
            {
                if (ae2.windDx * ae.windDx < 0)
                    ae.windCount = ae2.windCount;
                else
                    ae.windCount = ae2.windCount + ae.windDx;
            }

            ae.windCount2 = ae2.windCount2;
            ae2 = ae2.nextInAEL;
        }

        if (_fillrule == FillRule.EvenOdd)
            while (ae2 != ae)
            {
                if (GetPolyType(ae2!) != pt && !IsOpen(ae2!))
                    ae.windCount2 = (ae.windCount2 == 0 ? 1 : 0);
                ae2 = ae2!.nextInAEL;
            }
        else
            while (ae2 != ae)
            {
                if (GetPolyType(ae2!) != pt && !IsOpen(ae2!))
                    ae.windCount2 += ae2!.windDx;
                ae2 = ae2!.nextInAEL;
            }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetWindCountForOpenPathEdge(Active ae)
    {
        Active? ae2 = _actives;
        if (_fillrule == FillRule.EvenOdd)
        {
            int cnt1 = 0, cnt2 = 0;
            while (ae2 != ae)
            {
                if (GetPolyType(ae2!) == PathType.Clip)
                    cnt2++;
                else if (!IsOpen(ae2!))
                    cnt1++;
                ae2 = ae2!.nextInAEL;
            }

            ae.windCount = (IsOdd(cnt1) ? 1 : 0);
            ae.windCount2 = (IsOdd(cnt2) ? 1 : 0);
        }
        else
        {
            while (ae2 != ae)
            {
                if (GetPolyType(ae2!) == PathType.Clip)
                    ae.windCount2 += ae2!.windDx;
                else if (!IsOpen(ae2!))
                    ae.windCount += ae2!.windDx;
                ae2 = ae2!.nextInAEL;
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidAelOrder(Active resident, Active newcomer)
    {
        if (!Clipper.AlmostEqual(newcomer.curX, resident.curX))
            return Clipper.GreaterThan(newcomer.curX, resident.curX);

        var d = InternalClipper.CrossProduct(resident.top, newcomer.bot, newcomer.top);
        if (d != 0) return (d < 0);


        if (!IsMaxima(resident) && (resident.top.Y > newcomer.top.Y))
        {
            return InternalClipper.CrossProduct(newcomer.bot,
                resident.top, NextVertex(resident).pt) <= 0;
        }

        if (!IsMaxima(newcomer) && (newcomer.top.Y > resident.top.Y))
        {
            return InternalClipper.CrossProduct(newcomer.bot,
                newcomer.top, NextVertex(newcomer).pt) >= 0;
        }

        var y = newcomer.bot.Y;
        var newcomerIsLeft = newcomer.isLeftBound;

        if (!Clipper.AlmostEqual(resident.bot.Y, y) || !Clipper.AlmostEqual(resident.localMin.vertex.pt.Y, y))
            return newcomer.isLeftBound;
        if (resident.isLeftBound != newcomerIsLeft)
            return newcomerIsLeft;
        if (InternalClipper.IsCollinear(PrevPrevVertex(resident).pt,
                resident.bot, resident.top)) return true;
        return (InternalClipper.CrossProduct(PrevPrevVertex(resident).pt,
            newcomer.bot, PrevPrevVertex(newcomer).pt) > 0) == newcomerIsLeft;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void InsertLeftEdge(Active ae)
    {
        if (_actives == null)
        {
            ae.prevInAEL = null;
            ae.nextInAEL = null;
            _actives = ae;
        }
        else if (!IsValidAelOrder(_actives, ae))
        {
            ae.prevInAEL = null;
            ae.nextInAEL = _actives;
            _actives.prevInAEL = ae;
            _actives = ae;
        }
        else
        {
            var ae2 = _actives;
            while (ae2.nextInAEL != null && IsValidAelOrder(ae2.nextInAEL, ae))
                ae2 = ae2.nextInAEL;
            if (ae2.joinWith == JoinWith.Right) ae2 = ae2.nextInAEL!;
            ae.nextInAEL = ae2.nextInAEL;
            if (ae2.nextInAEL != null) ae2.nextInAEL.prevInAEL = ae;
            ae.prevInAEL = ae2;
            ae2.nextInAEL = ae;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void InsertRightEdge(Active ae, Active ae2)
    {
        ae2.nextInAEL = ae.nextInAEL;
        if (ae.nextInAEL != null) ae.nextInAEL.prevInAEL = ae2;
        ae2.prevInAEL = ae;
        ae.nextInAEL = ae2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddToHorzSegList(OutPt op)
    {
        if (op.outrec.isOpen) return;
        _horzSegList.Add(new HorzSegment(op));
    }

    private void InsertLocalMinimaIntoAEL(double botY)
    {
        while (HasLocMinAtY(botY))
        {
            var localMinima = PopLocalMinima();
            Active? leftBound;
            if ((localMinima.vertex.flags & VertexFlags.OpenStart) != VertexFlags.None)
            {
                leftBound = null;
            }
            else
            {
                leftBound = new Active
                {
                    bot = localMinima.vertex.pt,
                    curX = localMinima.vertex.pt.X,
                    windDx = -1,
                    vertexTop = localMinima.vertex.prev,
                    top = localMinima.vertex.prev!.pt,
                    outrec = null,
                    localMin = localMinima
                };
                SetDx(leftBound);
            }

            Active? rightBound;
            if ((localMinima.vertex.flags & VertexFlags.OpenEnd) != VertexFlags.None)
            {
                rightBound = null;
            }
            else
            {
                rightBound = new Active
                {
                    bot = localMinima.vertex.pt,
                    curX = localMinima.vertex.pt.X,
                    windDx = 1,
                    vertexTop = localMinima.vertex.next,
                    top = localMinima.vertex.next!.pt,
                    outrec = null,
                    localMin = localMinima
                };
                SetDx(rightBound);
            }

            if (leftBound != null && rightBound != null)
            {
                if (IsHorizontal(leftBound))
                {
                    if (IsHeadingRightHorz(leftBound)) SwapActives(ref leftBound, ref rightBound);
                }
                else if (IsHorizontal(rightBound))
                {
                    if (IsHeadingLeftHorz(rightBound)) SwapActives(ref leftBound, ref rightBound);
                }
                else if (leftBound.dx < rightBound.dx)
                    SwapActives(ref leftBound, ref rightBound);
            }
            else if (leftBound == null)
            {
                leftBound = rightBound;
                rightBound = null;
            }

            bool contributing;
            leftBound!.isLeftBound = true;
            InsertLeftEdge(leftBound);

            if (IsOpen(leftBound))
            {
                SetWindCountForOpenPathEdge(leftBound);
                contributing = IsContributingOpen(leftBound);
            }
            else
            {
                SetWindCountForClosedPathEdge(leftBound);
                contributing = IsContributingClosed(leftBound);
            }

            if (rightBound != null)
            {
                rightBound.windCount = leftBound.windCount;
                rightBound.windCount2 = leftBound.windCount2;
                InsertRightEdge(leftBound, rightBound);

                if (contributing)
                {
                    AddLocalMinPoly(leftBound, rightBound, leftBound.bot, true);
                    if (!IsHorizontal(leftBound))
                        CheckJoinLeft(leftBound, leftBound.bot);
                }

                while (rightBound.nextInAEL != null &&
                       IsValidAelOrder(rightBound.nextInAEL, rightBound))
                {
                    IntersectEdges(rightBound, rightBound.nextInAEL, rightBound.bot);
                    SwapPositionsInAEL(rightBound, rightBound.nextInAEL);
                }

                if (IsHorizontal(rightBound))
                    PushHorz(rightBound);
                else
                {
                    CheckJoinRight(rightBound, rightBound.bot);
                    InsertScanline(rightBound.top.Y);
                }
            }
            else if (contributing)
                StartOpenPath(leftBound, leftBound.bot);

            if (IsHorizontal(leftBound))
                PushHorz(leftBound);
            else
                InsertScanline(leftBound.top.Y);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PushHorz(Active ae)
    {
        ae.nextInSEL = _sel;
        _sel = ae;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool PopHorz(out Active? ae)
    {
        ae = _sel;
        if (_sel == null) return false;
        _sel = _sel.nextInSEL;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutPt AddLocalMinPoly(Active ae1, Active ae2, PointD pt, bool isNew = false)
    {
        OutRec outrec = NewOutRec();
        ae1.outrec = outrec;
        ae2.outrec = outrec;

        if (IsOpen(ae1))
        {
            outrec.owner = null;
            outrec.isOpen = true;
            if (ae1.windDx > 0)
                SetSides(outrec, ae1, ae2);
            else
                SetSides(outrec, ae2, ae1);
        }
        else
        {
            outrec.isOpen = false;
            var prevHotEdge = GetPrevHotEdge(ae1);

            if (prevHotEdge != null)
            {
                if (_using_polytree)
                    SetOwner(outrec, prevHotEdge.outrec!);
                outrec.owner = prevHotEdge.outrec;
                if (OutrecIsAscending(prevHotEdge) == isNew)
                    SetSides(outrec, ae2, ae1);
                else
                    SetSides(outrec, ae1, ae2);
            }
            else
            {
                outrec.owner = null;
                if (isNew)
                    SetSides(outrec, ae1, ae2);
                else
                    SetSides(outrec, ae2, ae1);
            }
        }

        OutPt op = new OutPt(pt, outrec);
        outrec.pts = op;
        return op;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutPt? AddLocalMaxPoly(Active ae1, Active ae2, PointD pt)
    {
        if (IsJoined(ae1)) Split(ae1, pt);
        if (IsJoined(ae2)) Split(ae2, pt);

        if (IsFront(ae1) == IsFront(ae2))
        {
            if (IsOpenEnd(ae1))
                SwapFrontBackSides(ae1.outrec!);
            else if (IsOpenEnd(ae2))
                SwapFrontBackSides(ae2.outrec!);
            else
            {
                _succeeded = false;
                return null;
            }
        }

        OutPt result = AddOutPt(ae1, pt);
        if (ae1.outrec == ae2.outrec)
        {
            OutRec outrec = ae1.outrec!;
            outrec.pts = result;

            if (_using_polytree)
            {
                Active? e = GetPrevHotEdge(ae1);
                if (e == null)
                    outrec.owner = null;
                else
                    SetOwner(outrec, e.outrec!);
            }
            UncoupleOutRec(ae1);
        }
        else if (IsOpen(ae1))
        {
            if (ae1.windDx < 0)
                JoinOutrecPaths(ae1, ae2);
            else
                JoinOutrecPaths(ae2, ae1);
        }
        else if (ae1.outrec!.idx < ae2.outrec!.idx)
            JoinOutrecPaths(ae1, ae2);
        else
            JoinOutrecPaths(ae2, ae1);
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void JoinOutrecPaths(Active ae1, Active ae2)
    {
        var p1Start = ae1.outrec!.pts!;
        var p2Start = ae2.outrec!.pts!;
        var p1End = p1Start.next!;
        var p2End = p2Start.next!;
        if (IsFront(ae1))
        {
            p2End.prev = p1Start;
            p1Start.next = p2End;
            p2Start.next = p1End;
            p1End.prev = p2Start;
            ae1.outrec.pts = p2Start;
            ae1.outrec.frontEdge = ae2.outrec.frontEdge;
            if (ae1.outrec.frontEdge != null)
                ae1.outrec.frontEdge!.outrec = ae1.outrec;
        }
        else
        {
            p1End.prev = p2Start;
            p2Start.next = p1End;
            p1Start.next = p2End;
            p2End.prev = p1Start;

            ae1.outrec.backEdge = ae2.outrec.backEdge;
            if (ae1.outrec.backEdge != null)
                ae1.outrec.backEdge!.outrec = ae1.outrec;
        }

        ae2.outrec.frontEdge = null;
        ae2.outrec.backEdge = null;
        ae2.outrec.pts = null;
        SetOwner(ae2.outrec, ae1.outrec);

        if (IsOpenEnd(ae1))
        {
            ae2.outrec.pts = ae1.outrec.pts;
            ae1.outrec.pts = null;
        }

        ae1.outrec = null;
        ae2.outrec = null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutPt AddOutPt(Active ae, PointD pt)
    {

        var outrec = ae.outrec!;
        var toFront = IsFront(ae);
        var opFront = outrec.pts!;
        var opBack = opFront.next!;

        switch (toFront)
        {
            case true when (pt == opFront.pt):
                return opFront;
            case false when (pt == opBack.pt):
                return opBack;
        }

        var newOp = new OutPt(pt, outrec);
        opBack.prev = newOp;
        newOp.prev = opFront;
        newOp.next = opBack;
        opFront.next = newOp;
        if (toFront) outrec.pts = newOp;
        return newOp;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutRec NewOutRec()
    {
        OutRec result = new OutRec
        {
            idx = _outrecList.Count
        };
        _outrecList.Add(result);
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutPt StartOpenPath(Active ae, PointD pt)
    {
        var outrec = NewOutRec();
        outrec.isOpen = true;
        if (ae.windDx > 0)
        {
            outrec.frontEdge = ae;
            outrec.backEdge = null;
        }
        else
        {
            outrec.frontEdge = null;
            outrec.backEdge = ae;
        }

        ae.outrec = outrec;
        var op = new OutPt(pt, outrec);
        outrec.pts = op;
        return op;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void UpdateEdgeIntoAEL(Active ae)
    {
        ae.bot = ae.top;
        ae.vertexTop = NextVertex(ae);
        ae.top = ae.vertexTop!.pt;
        ae.curX = ae.bot.X;
        SetDx(ae);

        if (IsJoined(ae)) Split(ae, ae.bot);

        if (IsHorizontal(ae))
        {
            if (!IsOpen(ae)) TrimHorz(ae, PreserveCollinear);
            return;
        }
        InsertScanline(ae.top.Y);

        CheckJoinLeft(ae, ae.bot);
        CheckJoinRight(ae, ae.bot, true);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Active? FindEdgeWithMatchingLocMin(Active e)
    {
        var result = e.nextInAEL;
        while (result != null)
        {
            if (result.localMin == e.localMin) return result;
            if (!IsHorizontal(result) && e.bot != result.bot) result = null;
            else result = result.nextInAEL;
        }
        result = e.prevInAEL;
        while (result != null)
        {
            if (result.localMin == e.localMin) return result;
            if (!IsHorizontal(result) && e.bot != result.bot) return null;
            result = result.prevInAEL;
        }
        return result;
    }

    private void IntersectEdges(Active ae1, Active ae2, PointD pt)
    {
        OutPt? resultOp = null;
        if (_hasOpenPaths && (IsOpen(ae1) || IsOpen(ae2)))
        {
            if (IsOpen(ae1) && IsOpen(ae2)) return;
            if (IsOpen(ae2)) SwapActives(ref ae1, ref ae2);
            if (IsJoined(ae2)) Split(ae2, pt);

            if (_cliptype == ClipType.Union)
            {
                if (!IsHotEdge(ae2)) return;
            }
            else if (ae2.localMin.polytype == PathType.Subject) return;

            switch (_fillrule)
            {
                case FillRule.Positive:
                    if (ae2.windCount != 1) return;
                    break;
                case FillRule.Negative:
                    if (ae2.windCount != -1) return;
                    break;
                default:
                    if (Math.Abs(ae2.windCount) != 1) return;
                    break;
            }

            if (IsHotEdge(ae1))
            {
                resultOp = AddOutPt(ae1, pt);

                if (IsFront(ae1))
                    ae1.outrec!.frontEdge = null;
                else
                    ae1.outrec!.backEdge = null;
                ae1.outrec = null;
            }

            else if (pt == ae1.localMin.vertex.pt &&
              !IsOpenEnd(ae1.localMin.vertex))
            {

                var ae3 = FindEdgeWithMatchingLocMin(ae1);
                if (ae3 != null && IsHotEdge(ae3))
                {
                    ae1.outrec = ae3.outrec;
                    if (ae1.windDx > 0)
                        SetSides(ae3.outrec!, ae1, ae3);
                    else
                        SetSides(ae3.outrec!, ae3, ae1);
                    return;
                }

                resultOp = StartOpenPath(ae1, pt);
            }
            else
                resultOp = StartOpenPath(ae1, pt);

            return;
        }

        if (IsJoined(ae1)) Split(ae1, pt);
        if (IsJoined(ae2)) Split(ae2, pt);


        int oldE1WindCount, oldE2WindCount;
        if (ae1.localMin.polytype == ae2.localMin.polytype)
        {
            if (_fillrule == FillRule.EvenOdd)
            {
                oldE1WindCount = ae1.windCount;
                ae1.windCount = ae2.windCount;
                ae2.windCount = oldE1WindCount;
            }
            else
            {
                if (ae1.windCount + ae2.windDx == 0)
                    ae1.windCount = -ae1.windCount;
                else
                    ae1.windCount += ae2.windDx;
                if (ae2.windCount - ae1.windDx == 0)
                    ae2.windCount = -ae2.windCount;
                else
                    ae2.windCount -= ae1.windDx;
            }
        }
        else
        {
            if (_fillrule != FillRule.EvenOdd)
                ae1.windCount2 += ae2.windDx;
            else
                ae1.windCount2 = (ae1.windCount2 == 0 ? 1 : 0);
            if (_fillrule != FillRule.EvenOdd)
                ae2.windCount2 -= ae1.windDx;
            else
                ae2.windCount2 = (ae2.windCount2 == 0 ? 1 : 0);
        }

        switch (_fillrule)
        {
            case FillRule.Positive:
                oldE1WindCount = ae1.windCount;
                oldE2WindCount = ae2.windCount;
                break;
            case FillRule.Negative:
                oldE1WindCount = -ae1.windCount;
                oldE2WindCount = -ae2.windCount;
                break;
            default:
                oldE1WindCount = Math.Abs(ae1.windCount);
                oldE2WindCount = Math.Abs(ae2.windCount);
                break;
        }

        var e1WindCountIs0or1 = oldE1WindCount is 0 or 1;
        var e2WindCountIs0or1 = oldE2WindCount is 0 or 1;

        if ((!IsHotEdge(ae1) && !e1WindCountIs0or1) ||
          (!IsHotEdge(ae2) && !e2WindCountIs0or1)) return;


        if (IsHotEdge(ae1) && IsHotEdge(ae2))
        {
            if ((oldE1WindCount != 0 && oldE1WindCount != 1) || (oldE2WindCount != 0 && oldE2WindCount != 1) ||
                (ae1.localMin.polytype != ae2.localMin.polytype && _cliptype != ClipType.Xor))
            {
                resultOp = AddLocalMaxPoly(ae1, ae2, pt);
            }
            else if (IsFront(ae1) || (ae1.outrec == ae2.outrec))
            {
                resultOp = AddLocalMaxPoly(ae1, ae2, pt);
                AddLocalMinPoly(ae1, ae2, pt);
            }
            else
            {
                resultOp = AddOutPt(ae1, pt);
                AddOutPt(ae2, pt);
                SwapOutrecs(ae1, ae2);
            }
        }

        else if (IsHotEdge(ae1))
        {
            resultOp = AddOutPt(ae1, pt);
            SwapOutrecs(ae1, ae2);
        }
        else if (IsHotEdge(ae2))
        {
            resultOp = AddOutPt(ae2, pt);
            SwapOutrecs(ae1, ae2);
        }

        else
        {
            double e1Wc2, e2Wc2;
            switch (_fillrule)
            {
                case FillRule.Positive:
                    e1Wc2 = ae1.windCount2;
                    e2Wc2 = ae2.windCount2;
                    break;
                case FillRule.Negative:
                    e1Wc2 = -ae1.windCount2;
                    e2Wc2 = -ae2.windCount2;
                    break;
                default:
                    e1Wc2 = Math.Abs(ae1.windCount2);
                    e2Wc2 = Math.Abs(ae2.windCount2);
                    break;
            }

            if (!IsSamePolyType(ae1, ae2))
            {
                resultOp = AddLocalMinPoly(ae1, ae2, pt);
            }
            else if (oldE1WindCount == 1 && oldE2WindCount == 1)
            {
                resultOp = null;
                switch (_cliptype)
                {
                    case ClipType.Union:
                        if (e1Wc2 > 0 && e2Wc2 > 0) return;
                        resultOp = AddLocalMinPoly(ae1, ae2, pt);
                        break;

                    case ClipType.Difference:
                        if (((GetPolyType(ae1) == PathType.Clip) && (e1Wc2 > 0) && (e2Wc2 > 0)) ||
                            ((GetPolyType(ae1) == PathType.Subject) && (e1Wc2 <= 0) && (e2Wc2 <= 0)))
                        {
                            resultOp = AddLocalMinPoly(ae1, ae2, pt);
                        }

                        break;

                    case ClipType.Xor:
                        resultOp = AddLocalMinPoly(ae1, ae2, pt);
                        break;

                    default:
                        if (e1Wc2 <= 0 || e2Wc2 <= 0) return;
                        resultOp = AddLocalMinPoly(ae1, ae2, pt);
                        break;
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DeleteFromAEL(Active ae)
    {
        var prev = ae.prevInAEL;
        var next = ae.nextInAEL;
        if (prev == null && next == null && (ae != _actives)) return;
        if (prev != null)
            prev.nextInAEL = next;
        else
            _actives = next;
        if (next != null) next.prevInAEL = prev;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AdjustCurrXAndCopyToSEL(double topY)
    {
        var ae = _actives;
        _sel = ae;
        while (ae != null)
        {
            ae.prevInSEL = ae.prevInAEL;
            ae.nextInSEL = ae.nextInAEL;
            ae.jump = ae.nextInSEL;
            ae.curX = ae.joinWith == JoinWith.Left ? ae.prevInAEL!.curX :
                TopX(ae, topY);
            ae = ae.nextInAEL;
        }
    }

    protected void ExecuteInternal(ClipType ct, FillRule fillRule)
    {
        if (ct == ClipType.NoClip) return;
        _fillrule = fillRule;
        _cliptype = ct;
        Reset();
        if (!PopScanline(out var y)) return;
        while (_succeeded)
        {
            InsertLocalMinimaIntoAEL(y);
            Active? ae;
            while (PopHorz(out ae)) DoHorizontal(ae!);
            if (_horzSegList.Count > 0)
            {
                ConvertHorzSegsToJoins();
                _horzSegList.Clear();
            }
            _currentBotY = y;
            if (!PopScanline(out y))
                break;
            DoIntersections(y);
            DoTopOfScanbeam(y);
            while (PopHorz(out ae)) DoHorizontal(ae!);
        }
        if (_succeeded) ProcessHorzJoins();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoIntersections(double topY)
    {
        if (!BuildIntersectList(topY)) return;
        ProcessIntersectList();
        DisposeIntersectNodes();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DisposeIntersectNodes()
    {
        _intersectList.Clear();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddNewIntersectNode(Active ae1, Active ae2, double topY)
    {
        if (!InternalClipper.GetSegmentIntersectPt(
                ae1.bot, ae1.top, ae2.bot, ae2.top, out PointD ip))
            ip = new PointD(ae1.curX, topY);

        if (ip.Y > _currentBotY || ip.Y < topY)
        {
            var absDx1 = Math.Abs(ae1.dx);
            var absDx2 = Math.Abs(ae2.dx);
            switch (absDx1 > 100)
            {
                case true when absDx2 > 100:
                    {
                        if (absDx1 > absDx2)
                            ip = InternalClipper.GetClosestPtOnSegment(ip, ae1.bot, ae1.top);
                        else
                            ip = InternalClipper.GetClosestPtOnSegment(ip, ae2.bot, ae2.top);
                        break;
                    }
                case true:
                    ip = InternalClipper.GetClosestPtOnSegment(ip, ae1.bot, ae1.top);
                    break;
                default:
                    {
                        if (absDx2 > 100)
                            ip = InternalClipper.GetClosestPtOnSegment(ip, ae2.bot, ae2.top);
                        else
                        {
                            if (ip.Y < topY) ip.Y = topY;
                            else ip.Y = _currentBotY;
                            if (absDx1 < absDx2) ip.X = TopX(ae1, ip.Y);
                            else ip.X = TopX(ae2, ip.Y);
                        }

                        break;
                    }
            }
        }
        var node = new IntersectNode(ip, ae1, ae2);
        _intersectList.Add(node);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Active? ExtractFromSEL(Active ae)
    {
        var res = ae.nextInSEL;
        if (res != null)
            res.prevInSEL = ae.prevInSEL;
        ae.prevInSEL!.nextInSEL = res;
        return res;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Insert1Before2InSEL(Active ae1, Active ae2)
    {
        ae1.prevInSEL = ae2.prevInSEL;
        if (ae1.prevInSEL != null)
            ae1.prevInSEL.nextInSEL = ae1;
        ae1.nextInSEL = ae2;
        ae2.prevInSEL = ae1;
    }

    private bool BuildIntersectList(double topY)
    {
        if (_actives?.nextInAEL == null) return false;

        AdjustCurrXAndCopyToSEL(topY);

        var left = _sel;

        while (left!.jump != null)
        {
            Active? prevBase = null;
            while (left?.jump != null)
            {
                var currBase = left;
                var right = left.jump;
                var lEnd = right;
                var rEnd = right.jump;
                left.jump = rEnd;
                while (left != lEnd && right != rEnd)
                {
                    if (right!.curX < left!.curX)
                    {
                        var tmp = right.prevInSEL!;
                        for (; ; )
                        {
                            AddNewIntersectNode(tmp, right, topY);
                            if (tmp == left) break;
                            tmp = tmp.prevInSEL!;
                        }

                        tmp = right;
                        right = ExtractFromSEL(tmp);
                        lEnd = right;
                        Insert1Before2InSEL(tmp, left);
                        if (left != currBase) continue;
                        currBase = tmp;
                        currBase.jump = rEnd;
                        if (prevBase == null) _sel = currBase;
                        else prevBase.jump = currBase;
                    }
                    else left = left.nextInSEL;
                }

                prevBase = currBase;
                left = rEnd;
            }
            left = _sel;
        }

        return _intersectList.Count > 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ProcessIntersectList()
    {
        _intersectList.Sort(new IntersectListSort());

        for (var i = 0; i < _intersectList.Count; ++i)
        {
            if (!EdgesAdjacentInAEL(_intersectList[i]))
            {
                var j = i + 1;
                while (!EdgesAdjacentInAEL(_intersectList[j])) j++;
                (_intersectList[j], _intersectList[i]) =
                    (_intersectList[i], _intersectList[j]);
            }

            IntersectNode node = _intersectList[i];
            IntersectEdges(node.edge1, node.edge2, node.pt);
            SwapPositionsInAEL(node.edge1, node.edge2);

            node.edge1.curX = node.pt.X;
            node.edge2.curX = node.pt.X;
            CheckJoinLeft(node.edge2, node.pt, true);
            CheckJoinRight(node.edge1, node.pt, true);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SwapPositionsInAEL(Active ae1, Active ae2)
    {
        var next = ae2.nextInAEL;
        if (next != null) next.prevInAEL = ae1;
        var prev = ae1.prevInAEL;
        if (prev != null) prev.nextInAEL = ae2;
        ae2.prevInAEL = prev;
        ae2.nextInAEL = ae1;
        ae1.prevInAEL = ae2;
        ae1.nextInAEL = next;
        if (ae2.prevInAEL == null) _actives = ae2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ResetHorzDirection(Active horz, Vertex? vertexMax,
        out double leftX, out double rightX)
    {
        if (horz.bot.X == horz.top.X)
        {
            leftX = horz.curX;
            rightX = horz.curX;
            var ae = horz.nextInAEL;
            while (ae != null && ae.vertexTop != vertexMax)
                ae = ae.nextInAEL;
            return ae != null;
        }

        if (horz.curX < horz.top.X)
        {
            leftX = horz.curX;
            rightX = horz.top.X;
            return true;
        }
        leftX = horz.top.X;
        rightX = horz.curX;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TrimHorz(Active horzEdge, bool preserveCollinear)
    {
        var wasTrimmed = false;
        var pt = NextVertex(horzEdge).pt;

        while (Clipper.AlmostEqual(pt.Y, horzEdge.top.Y))
        {
            if (preserveCollinear &&
                (Clipper.LessThan(pt.X, horzEdge.top.X)) != (Clipper.LessThan(horzEdge.bot.X, horzEdge.top.X)))
                break;

            horzEdge.vertexTop = NextVertex(horzEdge);
            horzEdge.top = pt;
            wasTrimmed = true;
            if (IsMaxima(horzEdge)) break;
            pt = NextVertex(horzEdge).pt;
        }
        if (wasTrimmed) SetDx(horzEdge);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutPt GetLastOp(Active hotEdge)
    {
        var outrec = hotEdge.outrec!;
        return (hotEdge == outrec.frontEdge) ?
            outrec.pts! : outrec.pts!.next!;
    }


    private void DoHorizontal(Active horz)
    {
        var horzIsOpen = IsOpen(horz);
        var Y = horz.bot.Y;

        var vertex_max = horzIsOpen ?
          GetCurrYMaximaVertex_Open(horz) :
          GetCurrYMaximaVertex(horz);

        bool isLeftToRight =
          ResetHorzDirection(horz, vertex_max, out var leftX, out var rightX);

        if (IsHotEdge(horz))
        {
            OutPt op = AddOutPt(horz, new PointD(horz.curX, Y));
            AddToHorzSegList(op);
        }

        for (; ; )
        {
            var ae = isLeftToRight ? horz.nextInAEL : horz.prevInAEL;

            while (ae != null)
            {
                if (ae.vertexTop == vertex_max)
                {
                    if (IsHotEdge(horz) && IsJoined(ae)) Split(ae, ae.top);

                    if (IsHotEdge(horz))
                    {
                        while (horz.vertexTop != vertex_max)
                        {
                            AddOutPt(horz, horz.top);
                            UpdateEdgeIntoAEL(horz);
                        }
                        if (isLeftToRight)
                            AddLocalMaxPoly(horz, ae, horz.top);
                        else
                            AddLocalMaxPoly(ae, horz, horz.top);
                    }
                    DeleteFromAEL(ae);
                    DeleteFromAEL(horz);
                    return;
                }

                PointD pt;
                if (vertex_max != horz.vertexTop || IsOpenEnd(horz))
                {
                    if ((isLeftToRight && Clipper.GreaterThan(ae.curX, rightX)) ||
                        (!isLeftToRight && Clipper.LessThan(ae.curX, leftX))) break;

                    if (Clipper.AlmostEqual(ae.curX, horz.top.X) && !IsHorizontal(ae))
                    {
                        pt = NextVertex(horz).pt;
                        if (IsOpen(ae) && !IsSamePolyType(ae, horz) && !IsHotEdge(ae))
                        {

                            if ((isLeftToRight && (Clipper.GreaterThan(TopX(ae, pt.Y), pt.X))) ||
                              (!isLeftToRight && (Clipper.LessThan(TopX(ae, pt.Y), pt.X)))) break;
                        }

                        else if ((isLeftToRight && (Clipper.GreaterThan(TopX(ae, pt.Y), pt.X)) ||
                                 (!isLeftToRight && (Clipper.LessThan(TopX(ae, pt.Y), pt.X)))))
                        {
                            break;
                        }
                    }
                }

                pt = new PointD(ae.curX, Y);

                if (isLeftToRight)
                {
                    IntersectEdges(horz, ae, pt);
                    SwapPositionsInAEL(horz, ae);
                    CheckJoinLeft(ae, pt);
                    horz.curX = ae.curX;
                    ae = horz.nextInAEL;
                }
                else
                {
                    IntersectEdges(ae, horz, pt);
                    SwapPositionsInAEL(ae, horz);
                    CheckJoinRight(ae, pt);
                    horz.curX = ae.curX;
                    ae = horz.prevInAEL;
                }

                if (IsHotEdge(horz))
                    AddToHorzSegList(GetLastOp(horz));

            }
            if (horzIsOpen && IsOpenEnd(horz))
            {
                if (IsHotEdge(horz))
                {
                    AddOutPt(horz, horz.top);
                    if (IsFront(horz))
                        horz.outrec!.frontEdge = null;
                    else
                        horz.outrec!.backEdge = null;
                    horz.outrec = null;
                }
                DeleteFromAEL(horz);
                return;
            }
            if (!Clipper.AlmostEqual(NextVertex(horz).pt.Y, horz.top.Y))
                break;

            if (IsHotEdge(horz))
                AddOutPt(horz, horz.top);

            UpdateEdgeIntoAEL(horz);

            isLeftToRight = ResetHorzDirection(horz,
              vertex_max, out leftX, out rightX);

        }

        if (IsHotEdge(horz))
        {
            var op = AddOutPt(horz, horz.top);
            AddToHorzSegList(op);
        }

        UpdateEdgeIntoAEL(horz);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoTopOfScanbeam(double y)
    {
        _sel = null;
        var ae = _actives;
        while (ae != null)
        {
            if (ae.top.Y == y)
            {
                ae.curX = ae.top.X;
                if (IsMaxima(ae))
                {
                    ae = DoMaxima(ae);
                    continue;
                }

                if (IsHotEdge(ae))
                    AddOutPt(ae, ae.top);
                UpdateEdgeIntoAEL(ae);
                if (IsHorizontal(ae))
                    PushHorz(ae);
            }
            else
                ae.curX = TopX(ae, y);

            ae = ae.nextInAEL;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Active? DoMaxima(Active ae)
    {
        var prevE = ae.prevInAEL;
        var nextE = ae.nextInAEL;

        if (IsOpenEnd(ae))
        {
            if (IsHotEdge(ae)) AddOutPt(ae, ae.top);
            if (IsHorizontal(ae)) return nextE;
            if (IsHotEdge(ae))
            {
                if (IsFront(ae))
                    ae.outrec!.frontEdge = null;
                else
                    ae.outrec!.backEdge = null;
                ae.outrec = null;
            }
            DeleteFromAEL(ae);
            return nextE;
        }

        var maxPair = GetMaximaPair(ae);
        if (maxPair == null) return nextE;

        if (IsJoined(ae)) Split(ae, ae.top);
        if (IsJoined(maxPair)) Split(maxPair, maxPair.top);

        while (nextE != maxPair)
        {
            IntersectEdges(ae, nextE!, ae.top);
            SwapPositionsInAEL(ae, nextE!);
            nextE = ae.nextInAEL;
        }

        if (IsOpen(ae))
        {
            if (IsHotEdge(ae))
                AddLocalMaxPoly(ae, maxPair, ae.top);
            DeleteFromAEL(maxPair);
            DeleteFromAEL(ae);
            return (prevE != null ? prevE.nextInAEL : _actives);
        }

        if (IsHotEdge(ae))
            AddLocalMaxPoly(ae, maxPair, ae.top);

        DeleteFromAEL(ae);
        DeleteFromAEL(maxPair);
        return (prevE != null ? prevE.nextInAEL : _actives);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsJoined(Active e)
    {
        return e.joinWith != JoinWith.None;
    }

    private void Split(Active e, PointD currPt)
    {
        if (e.joinWith == JoinWith.Right)
        {
            e.joinWith = JoinWith.None;
            e.nextInAEL!.joinWith = JoinWith.None;
            AddLocalMinPoly(e, e.nextInAEL, currPt, true);
        }
        else
        {
            e.joinWith = JoinWith.None;
            e.prevInAEL!.joinWith = JoinWith.None;
            AddLocalMinPoly(e.prevInAEL, e, currPt, true);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CheckJoinLeft(Active e,
        PointD pt, bool checkCurrX = false)
    {
        var prev = e.prevInAEL;
        if (prev == null ||
            !IsHotEdge(e) || !IsHotEdge(prev) ||
            IsHorizontal(e) || IsHorizontal(prev) ||
            IsOpen(e) || IsOpen(prev)) return;
        if ((Clipper.LessThan(pt.Y, e.top.Y + 2) || Clipper.LessThan(pt.Y, prev.top.Y + 2)) &&
            (Clipper.GreaterThan(e.bot.Y, pt.Y) || Clipper.GreaterThan(prev.bot.Y, pt.Y)))
        {
            return;
        }

        if (checkCurrX)
        {
            if (Clipper.PerpendicDistFromLineSqrd(pt, prev.bot, prev.top) > 0.25) return;
        }
        else if (!Clipper.AlmostEqual(e.curX, prev.curX)) return;
        if (!InternalClipper.IsCollinear(e.top, pt, prev.top)) return;

        if (e.outrec!.idx == prev.outrec!.idx)
            AddLocalMaxPoly(prev, e, pt);
        else if (e.outrec.idx < prev.outrec.idx)
            JoinOutrecPaths(e, prev);
        else
            JoinOutrecPaths(prev, e);
        prev.joinWith = JoinWith.Right;
        e.joinWith = JoinWith.Left;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CheckJoinRight(Active e,
        PointD pt, bool checkCurrX = false)
    {
        var next = e.nextInAEL;
        if (next == null ||
            !IsHotEdge(e) || !IsHotEdge(next) ||
            IsHorizontal(e) || IsHorizontal(next) ||
            IsOpen(e) || IsOpen(next)) return;
        if ((Clipper.LessThan(pt.Y, e.top.Y + 2) ||
            Clipper.LessThan(pt.Y, next.top.Y + 2)) &&
            (Clipper.GreaterThan(e.bot.Y, pt.Y) ||
            Clipper.GreaterThan(next.bot.Y, pt.Y)))
        {
            return;
        }

        if (checkCurrX)
        {
            if (Clipper.PerpendicDistFromLineSqrd(pt, next.bot, next.top) > 0.25) return;
        }
        else if (!Clipper.AlmostEqual(e.curX, next.curX)) return;
        if (!InternalClipper.IsCollinear(e.top, pt, next.top)) return;

        if (e.outrec!.idx == next.outrec!.idx)
            AddLocalMaxPoly(e, next, pt);
        else if (e.outrec.idx < next.outrec.idx)
            JoinOutrecPaths(e, next);
        else
            JoinOutrecPaths(next, e);
        e.joinWith = JoinWith.Right;
        next.joinWith = JoinWith.Left;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void FixOutRecPts(OutRec outrec)
    {
        var op = outrec.pts!;
        do
        {
            op.outrec = outrec;
            op = op.next!;
        } while (op != outrec.pts);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SetHorzSegHeadingForward(HorzSegment hs, OutPt opP, OutPt opN)
    {
        if (Clipper.AlmostEqual(opP.pt.X, opN.pt.X)) return false;
        if (Clipper.LessThan(opP.pt.X, opN.pt.X))
        {
            hs.leftOp = opP;
            hs.rightOp = opN;
            hs.leftToRight = true;
        }
        else
        {
            hs.leftOp = opN;
            hs.rightOp = opP;
            hs.leftToRight = false;
        }
        return true;
    }

    private static bool UpdateHorzSegment(HorzSegment hs)
    {
        var op = hs.leftOp!;
        var outrec = GetRealOutRec(op.outrec)!;
        var outrecHasEdges = outrec.frontEdge != null;
        var curr_y = op.pt.Y;
        OutPt opP = op, opN = op;
        if (outrecHasEdges)
        {
            OutPt opA = outrec.pts!, opZ = opA.next!;
            while (opP != opZ && Clipper.AlmostEqual(opP.prev.pt.Y, curr_y))
                opP = opP.prev;
            while (opN != opA && Clipper.AlmostEqual(opN.next!.pt.Y, curr_y))
                opN = opN.next;
        }
        else
        {
            while (opP.prev != opN && Clipper.AlmostEqual(opP.prev.pt.Y, curr_y))
                opP = opP.prev;
            while (opN.next != opP && Clipper.AlmostEqual(opN.next!.pt.Y, curr_y))
                opN = opN.next;
        }
        var result =
            SetHorzSegHeadingForward(hs, opP, opN) &&
            hs.leftOp!.horz == null;

        if (result)
            hs.leftOp!.horz = hs;
        else
            hs.rightOp = null;
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutPt DuplicateOp(OutPt op, bool insert_after)
    {
        var result = new OutPt(op.pt, op.outrec);
        if (insert_after)
        {
            result.next = op.next;
            result.next!.prev = result;
            result.prev = op;
            op.next = result;
        }
        else
        {
            result.prev = op.prev;
            result.prev.next = result;
            result.next = op;
            op.prev = result;
        }
        return result;
    }

    private static int HorzSegSort(HorzSegment? hs1, HorzSegment? hs2)
    {
        if (hs1 == null || hs2 == null) return 0;
        if (hs1.rightOp == null)
        {
            return hs2.rightOp == null ? 0 : 1;
        }
        if (hs2.rightOp == null)
            return -1;
        return hs1.leftOp!.pt.X.CompareTo(hs2.leftOp!.pt.X);
    }

    private void ConvertHorzSegsToJoins()
    {
        var k = 0;
        for (var index = 0; index < _horzSegList.Count; index++)
        {
            if (UpdateHorzSegment(_horzSegList[index]))
                k++;
        }
        if (k < 2) return;

        _horzSegList.Sort(HorzSegSort);

        for (var i = 0; i < k - 1; i++)
        {
            var hs1 = _horzSegList[i];
            for (var j = i + 1; j < k; j++)
            {
                var hs2 = _horzSegList[j];

                if ((Clipper.GreaterThanOrEqual(hs2.leftOp!.pt.X, hs1.rightOp!.pt.X)) ||
                    (hs2.leftToRight == hs1.leftToRight) ||
                    (Clipper.LessThanOrEqual(hs2.rightOp!.pt.X, hs1.leftOp!.pt.X)))
                    continue;

                double curr_y = hs1.leftOp.pt.Y;

                if (hs1.leftToRight)
                {
                    while (Clipper.AlmostEqual(hs1.leftOp.next!.pt.Y, curr_y) &&
                           (Clipper.LessThanOrEqual(hs1.leftOp.next.pt.X, hs2.leftOp.pt.X)))
                    {
                        hs1.leftOp = hs1.leftOp.next;
                    }

                    while (Clipper.AlmostEqual(hs2.leftOp.prev.pt.Y, curr_y) &&
                           (Clipper.LessThanOrEqual(hs2.leftOp.prev.pt.X, hs1.leftOp.pt.X)))
                    {
                        hs2.leftOp = hs2.leftOp.prev;
                    }

                    var join = new HorzJoin(
                        DuplicateOp(hs1.leftOp, true),
                        DuplicateOp(hs2.leftOp, false));
                    _horzJoinList.Add(join);
                }
                else
                {
                    while (Clipper.AlmostEqual(hs1.leftOp.prev.pt.Y, curr_y) &&
                           (Clipper.LessThanOrEqual(hs1.leftOp.prev.pt.X, hs2.leftOp.pt.X)))
                    {
                        hs1.leftOp = hs1.leftOp.prev;
                    }

                    while (Clipper.AlmostEqual(hs2.leftOp.next!.pt.Y, curr_y) &&
                           (Clipper.LessThanOrEqual(hs2.leftOp.next.pt.X, hs1.leftOp.pt.X)))
                    {
                        hs2.leftOp = hs2.leftOp.next;
                    }

                    var join = new HorzJoin(
                        DuplicateOp(hs2.leftOp, true),
                        DuplicateOp(hs1.leftOp, false));
                    _horzJoinList.Add(join);
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PathD GetCleanPath(OutPt op)
    {
        var result = new PathD();
        var op2 = op;

        while (op2.next != op &&
               ((Clipper.AlmostEqual(op2.pt.X, op2.next!.pt.X) && Clipper.AlmostEqual(op2.pt.X, op2.prev.pt.X)) ||
                (Clipper.AlmostEqual(op2.pt.Y, op2.next.pt.Y) && Clipper.AlmostEqual(op2.pt.Y, op2.prev.pt.Y))))
        {
            op2 = op2.next;
        }

        result.Add(op2.pt);
        var prevOp = op2;
        op2 = op2.next;

        while (op2 != op)
        {
            if ((!Clipper.AlmostEqual(op2.pt.X, op2.next!.pt.X) || !Clipper.AlmostEqual(op2.pt.X, prevOp.pt.X)) &&
                (!Clipper.AlmostEqual(op2.pt.Y, op2.next.pt.Y) || !Clipper.AlmostEqual(op2.pt.Y, prevOp.pt.Y)))
            {
                result.Add(op2.pt);
                prevOp = op2;
            }
            op2 = op2.next;
        }

        return result;
    }

    private static PointInPolygonResult PointInOpPolygon(PointD pt, OutPt op)
    {
        if (op == op.next || op.prev == op.next)
            return PointInPolygonResult.IsOutside;

        var op2 = op;
        do
        {
            if (!Clipper.AlmostEqual(op.pt.Y, pt.Y)) break;
            op = op.next!;
        } while (op != op2);

        if (Clipper.AlmostEqual(op.pt.Y, pt.Y))
            return PointInPolygonResult.IsOutside;

        bool isAbove = Clipper.LessThan(op.pt.Y, pt.Y), startingAbove = isAbove;
        var val = 0;

        op2 = op.next!;
        while (op2 != op)
        {
            if (isAbove)
            {
                while (op2 != op && Clipper.LessThan(op2.pt.Y, pt.Y))
                    op2 = op2.next!;
            }
            else
            {
                while (op2 != op && Clipper.GreaterThan(op2.pt.Y, pt.Y))
                    op2 = op2.next!;
            }

            if (op2 == op) break;

            if (Clipper.AlmostEqual(op2.pt.Y, pt.Y))
            {
                if (Clipper.AlmostEqual(op2.pt.X, pt.X) ||
                    (Clipper.AlmostEqual(op2.pt.Y, op2.prev.pt.Y) &&
                    (Clipper.LessThan(pt.X, op2.prev.pt.X)) != Clipper.LessThan(pt.X , op2.pt.X)))
                    return PointInPolygonResult.IsOn;

                op2 = op2.next!;
                if (op2 == op) break;
                continue;
            }

            if (Clipper.LessThanOrEqual(op2.pt.X, pt.X) || Clipper.LessThanOrEqual(op2.prev.pt.X, pt.X))
            {
                if ((Clipper.LessThan(op2.prev.pt.X, pt.X) && Clipper.LessThan(op2.pt.X, pt.X)))
                {
                    val = 1 - val;
                }
                else
                {
                    var d = InternalClipper.CrossProduct(op2.prev.pt, op2.pt, pt);
                    if (Clipper.AlmostEqual(d, 0)) return PointInPolygonResult.IsOn;
                    if ((Clipper.LessThan(d, 0)) == isAbove) val = 1 - val;
                }
            }
            isAbove = !isAbove;
            op2 = op2.next!;
        }

        if (isAbove == startingAbove) return val == 0 ? PointInPolygonResult.IsOutside : PointInPolygonResult.IsInside;

        var dFinal = InternalClipper.CrossProduct(op2.prev.pt, op2.pt, pt);
        if (Clipper.AlmostEqual(dFinal, 0)) return PointInPolygonResult.IsOn;
        if ((Clipper.LessThan(dFinal, 0)) == isAbove) val = 1 - val;

        return val == 0 ? PointInPolygonResult.IsOutside : PointInPolygonResult.IsInside;
    }

    private static bool Path1InsidePath2(OutPt op1, OutPt op2)
    {
        var outside_cnt = 0;
        var op = op1;
        do
        {
            var result = PointInOpPolygon(op.pt, op2);
            switch (result)
            {
                case PointInPolygonResult.IsOutside:
                    ++outside_cnt;
                    break;
                case PointInPolygonResult.IsInside:
                    --outside_cnt;
                    break;
            }
            op = op.next!;
        } while (op != op1 && Math.Abs(outside_cnt) < 2);
        if (Math.Abs(outside_cnt) > 1) return (outside_cnt < 0);
        PointD mp = GetBounds(GetCleanPath(op1)).MidPoint();
        PathD path2 = GetCleanPath(op2);
        return InternalClipper.PointInPolygon(mp, path2) != PointInPolygonResult.IsOutside;
    }

    private static void MoveSplits(OutRec fromOr, OutRec toOr)
    {
        if (fromOr.splits == null) return;
        toOr.splits ??= new List<int>();
        foreach (var i in fromOr.splits)
            toOr.splits.Add(i);
        fromOr.splits = null;
    }

    private void ProcessHorzJoins()
    {
        foreach (HorzJoin j in _horzJoinList)
        {
            var or1 = GetRealOutRec(j.op1!.outrec)!;
            var or2 = GetRealOutRec(j.op2!.outrec)!;

            var op1b = j.op1.next!;
            var op2b = j.op2.prev;
            j.op1.next = j.op2;
            j.op2.prev = j.op1;
            op1b.prev = op2b;
            op2b.next = op1b;

            if (or1 == or2)
            {
                or2 = NewOutRec();
                or2.pts = op1b;
                FixOutRecPts(or2);

                if (or1.pts!.outrec == or2)
                {
                    or1.pts = j.op1;
                    or1.pts.outrec = or1;
                }

                if (_using_polytree)
                {
                    if (Path1InsidePath2(or1.pts, or2.pts))
                    {
                        (or2.pts, or1.pts) = (or1.pts, or2.pts);
                        FixOutRecPts(or1);
                        FixOutRecPts(or2);
                        or2.owner = or1;
                    }
                    else if (Path1InsidePath2(or2.pts, or1.pts))
                        or2.owner = or1;
                    else
                        or2.owner = or1.owner;

                    or1.splits ??= new List<int>();
                    or1.splits.Add(or2.idx);
                }
                else
                    or2.owner = or1;
            }
            else
            {
                or2.pts = null;
                if (_using_polytree)
                {
                    SetOwner(or2, or1);
                    MoveSplits(or2, or1);
                }
                else
                    or2.owner = or1;
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool PtsReallyClose(PointD pt1, PointD pt2)
    {
        return (Math.Abs(pt1.X - pt2.X) < Epsilon.GetEpsilonValue()) && (Math.Abs(pt1.Y - pt2.Y) < Epsilon.GetEpsilonValue());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsVerySmallTriangle(OutPt op)
    {
        return op.next!.next == op.prev &&
               (PtsReallyClose(op.prev.pt, op.next.pt) ||
                PtsReallyClose(op.pt, op.next.pt) ||
                PtsReallyClose(op.pt, op.prev.pt));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidClosedPath(OutPt? op)
    {
        return (op != null && op.next != op &&
                (op.next != op.prev || !IsVerySmallTriangle(op)));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutPt? DisposeOutPt(OutPt op)
    {
        OutPt? result = (op.next == op ? null : op.next);
        op.prev.next = op.next;
        op.next!.prev = op.prev;
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CleanCollinear(OutRec? outrec)
    {
        outrec = GetRealOutRec(outrec);

        if (outrec == null || outrec.isOpen) return;

        if (!IsValidClosedPath(outrec.pts))
        {
            outrec.pts = null;
            return;
        }

        var startOp = outrec.pts!;
        var op2 = startOp;
        for (; ; )
        {
            if (InternalClipper.IsCollinear(op2!.prev.pt, op2.pt, op2.next!.pt) &&
                ((Clipper.AlmostEqual(op2.pt.X, op2.prev.pt.X) && Clipper.AlmostEqual(op2.pt.Y, op2.prev.pt.Y))
                 || (Clipper.AlmostEqual(op2.pt.X, op2.next.pt.X) && Clipper.AlmostEqual(op2.pt.Y, op2.next.pt.Y))
                 || !PreserveCollinear
                 || Clipper.LessThan(InternalClipper.DotProduct(op2.prev.pt, op2.pt, op2.next.pt), 0)))
            {
                if (op2 == outrec.pts)
                    outrec.pts = op2.prev;
                op2 = DisposeOutPt(op2);
                if (!IsValidClosedPath(op2))
                {
                    outrec.pts = null;
                    return;
                }
                startOp = op2!;
                continue;
            }
            op2 = op2.next;
            if (op2 == startOp) break;
        }
        FixSelfIntersects(outrec);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoSplitOp(OutRec outrec, OutPt splitOp)
    {
        var prevOp = splitOp.prev;
        var nextNextOp = splitOp.next!.next!;
        outrec.pts = prevOp;

        InternalClipper.GetSegmentIntersectPt(
            prevOp.pt, splitOp.pt, splitOp.next.pt, nextNextOp.pt, out PointD ip);


        var area1 = Area(prevOp);
        var absArea1 = Math.Abs(area1);

        if (absArea1 < 2)
        {
            outrec.pts = null;
            return;
        }

        var area2 = AreaTriangle(ip, splitOp.pt, splitOp.next.pt);
        var absArea2 = Math.Abs(area2);


        if (Clipper.AlmostEqual(ip.X, prevOp.pt.X) && Clipper.AlmostEqual(ip.Y, prevOp.pt.Y) ||
            Clipper.AlmostEqual(ip.X, nextNextOp.pt.X) && Clipper.AlmostEqual(ip.Y, nextNextOp.pt.Y))
        {
            nextNextOp.prev = prevOp;
            prevOp.next = nextNextOp;
        }
        else
        {
            var newOp2 = new OutPt(ip, outrec) { prev = prevOp, next = nextNextOp };
            nextNextOp.prev = newOp2;
            prevOp.next = newOp2;
        }

        if (!(absArea2 > 1) ||
            (!(absArea2 > absArea1) &&
             ((area2 > 0) != (area1 > 0)))) return;
        var newOutRec = NewOutRec();
        newOutRec.owner = outrec.owner;
        splitOp.outrec = newOutRec;
        splitOp.next.outrec = newOutRec;

        var newOp = new OutPt(ip, newOutRec) { prev = splitOp.next, next = splitOp };
        newOutRec.pts = newOp;
        splitOp.prev = newOp;
        splitOp.next.next = newOp;

        if (!_using_polytree) return;
        if (Path1InsidePath2(prevOp, newOp))
        {
            newOutRec.splits ??= new List<int>();
            newOutRec.splits.Add(outrec.idx);
        }
        else
        {
            outrec.splits ??= new List<int>();
            outrec.splits.Add(newOutRec.idx);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void FixSelfIntersects(OutRec outrec)
    {
        var op2 = outrec.pts!;
        if (op2.prev == op2.next!.next) return;
        for (; ; )
        {
            if (InternalClipper.SegsIntersect(op2.prev.pt,
                    op2.pt, op2.next!.pt, op2.next.next!.pt))
            {
                DoSplitOp(outrec, op2);
                if (outrec.pts == null) return;
                op2 = outrec.pts;
                if (op2.prev == op2.next!.next) break;
                continue;
            }

            op2 = op2.next;
            if (op2 == outrec.pts) break;
        }
    }

    internal static bool BuildPath(OutPt? op, bool reverse, bool isOpen, PathD path)
    {
        if (op == null || op.next == op || (!isOpen && op.next == op.prev)) return false;
        path.Clear();

        PointD lastPt;
        OutPt op2;
        if (reverse)
        {
            lastPt = op.pt;
            op2 = op.prev;
        }
        else
        {
            op = op.next!;
            lastPt = op.pt;
            op2 = op.next!;
        }
        path.Add(lastPt);

        while (op2 != op)
        {
            if (op2.pt != lastPt)
            {
                lastPt = op2.pt;
                path.Add(lastPt);
            }
            if (reverse)
                op2 = op2.prev;
            else
                op2 = op2.next!;
        }

        return path.Count != 3 || isOpen || !IsVerySmallTriangle(op2);
    }

    protected bool BuildPaths(PathsD solutionClosed, PathsD solutionOpen)
    {
        solutionClosed.Clear();
        solutionOpen.Clear();
        solutionClosed.EnsureCapacity(_outrecList.Count);
        solutionOpen.EnsureCapacity(_outrecList.Count);

        var i = 0;
        while (i < _outrecList.Count)
        {
            var outrec = _outrecList[i++];
            if (outrec.pts == null) continue;

            PathD path = new();
            if (outrec.isOpen)
            {
                if (BuildPath(outrec.pts, ReverseSolution, true, path))
                    solutionOpen.Add(path);
            }
            else
            {
                CleanCollinear(outrec);
                if (BuildPath(outrec.pts, ReverseSolution, false, path))
                    solutionClosed.Add(path);
            }
        }
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static RectD GetBounds(PathD path)
    {
        if (path.Count == 0) return new RectD();
        var result = Clipper.InvalidRectD;
        foreach (var pt in path)
        {
            if (Clipper.LessThan(pt.X, result.Left)) result.Left = pt.X;
            if (Clipper.GreaterThan(pt.X, result.Right)) result.Right = pt.X;
            if (Clipper.LessThan(pt.Y, result.Top)) result.Top = pt.Y;
            if (Clipper.GreaterThan(pt.Y, result.Bottom)) result.Bottom = pt.Y;
        }
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool CheckBounds(OutRec outrec)
    {
        if (outrec.pts == null) return false;
        if (!outrec.bounds.IsEmpty()) return true;
        CleanCollinear(outrec);
        if (outrec.pts == null ||
            !BuildPath(outrec.pts, ReverseSolution, false, outrec.path))
            return false;
        outrec.bounds = GetBounds(outrec.path);
        return true;
    }

    private bool CheckSplitOwner(OutRec outrec, List<int>? splits)
    {
        for (var i = 0; i < splits!.Count; i++)
        {
            var split = _outrecList[i];
            if (split.pts == null && split.splits != null &&
                CheckSplitOwner(outrec, split.splits)) return true;
            split = GetRealOutRec(split);
            if (split == null || split == outrec || split.recursiveSplit == outrec) continue;
            split.recursiveSplit = outrec;
            if (split.splits != null && CheckSplitOwner(outrec, split.splits)) return true;
            if (!IsValidOwner(outrec, split) ||
                !CheckBounds(split) ||
                !split.bounds.Contains(outrec.bounds) ||
                !Path1InsidePath2(outrec.pts!, split.pts!)) continue;
            outrec.owner = split;
            return true;
        }
        return false;
    }

    private void RecursiveCheckOwners(OutRec outrec, PolyPathBase polypath)
    {
        if (outrec.polypath != null || outrec.bounds.IsEmpty()) return;

        while (outrec.owner != null)
        {
            if (outrec.owner.splits != null &&
                CheckSplitOwner(outrec, outrec.owner.splits)) break;
            if (outrec.owner.pts != null && CheckBounds(outrec.owner) &&
                Path1InsidePath2(outrec.pts!, outrec.owner.pts!)) break;
            outrec.owner = outrec.owner.owner;
        }

        if (outrec.owner != null)
        {
            if (outrec.owner.polypath == null)
                RecursiveCheckOwners(outrec.owner, polypath);
            outrec.polypath = outrec.owner.polypath!.AddChild(outrec.path);
        }
        else
            outrec.polypath = polypath.AddChild(outrec.path);
    }

    protected void BuildTree(PolyPathBase polytree, PathsD solutionOpen)
    {
        polytree.Clear();
        solutionOpen.Clear();
        if (_hasOpenPaths)
            solutionOpen.EnsureCapacity(_outrecList.Count);

        var i = 0;
        while (i < _outrecList.Count)
        {
            var outrec = _outrecList[i++];
            if (outrec.pts == null) continue;

            if (outrec.isOpen)
            {
                var open_path = new PathD();
                if (BuildPath(outrec.pts, ReverseSolution, true, open_path))
                    solutionOpen.Add(open_path);
                continue;
            }
            if (CheckBounds(outrec))
                RecursiveCheckOwners(outrec, polytree);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public RectD GetBounds()
    {
        var bounds = Clipper.InvalidRectD;
        for (var i = 0; i < _vertexList.Count; i++)
        {
            var v = _vertexList[i];
            do
            {
                if (Clipper.LessThan(v.pt.X, bounds.Left)) bounds.Left = v.pt.X;
                if (Clipper.GreaterThan(v.pt.X, bounds.Right)) bounds.Right = v.pt.X;
                if (Clipper.LessThan(v.pt.Y, bounds.Top)) bounds.Top = v.pt.Y;
                if (Clipper.GreaterThan(v.pt.Y, bounds.Bottom)) bounds.Bottom = v.pt.Y;
                v = v.next!;
            } while (v != _vertexList[i]);
        }
        return bounds.IsEmpty() ? new RectD(0, 0, 0, 0) : bounds;
    }

}


