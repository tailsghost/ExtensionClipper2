using System.IO;
using System.Runtime.CompilerServices;
using ExtensionClipper2.Core;
using ExtensionClipper2.Engine;
using ExtensionClipper2.Enums;
using ExtensionClipper2.RectClips.Property;

namespace ExtensionClipper2.RectClips;

internal class RectClip
{
    readonly protected RectD rect_;
    readonly protected PointD mp_;
    readonly protected PathD rectPath_;
    protected RectD pathBounds_;
    protected List<OutPt2?> results_;
    protected List<OutPt2?>[] edges_;
    protected int currIdx_;

    internal RectClip(RectD rect)
    {
        currIdx_ = -1;
        rect_ = rect;
        mp_ = rect.MidPoint();
        rectPath_ = rect_.AsPath();
        results_ = new List<OutPt2?>();
        edges_ = new List<OutPt2?>[8];
        for (int i = 0; i < 8; i++)
            edges_[i] = new List<OutPt2?>();
    }

    internal OutPt2 Add(PointD pt, bool startingNewPath = false)
    { 
        var currIdx = results_.Count;
        OutPt2 result;
        if ((currIdx == 0) || startingNewPath)
        {
            result = new OutPt2(pt);
            results_.Add(result);
            result.ownerIdx = currIdx;
            result.prev = result;
            result.next = result;
        }
        else
        {
            currIdx--;
            OutPt2? prevOp = results_[currIdx];
            if (prevOp!.pt == pt) return prevOp;
            result = new OutPt2(pt)
            {
                ownerIdx = currIdx,
                next = prevOp.next
            };
            prevOp.next!.prev = result;
            prevOp.next = result;
            result.prev = prevOp;
            results_[currIdx] = result;
        }
        return result;
    }

    private static bool Path1ContainsPath2(PathD path1, PathD path2)
    {
        var ioCount = 0;

        for (var i = 0; i < path2.Count; i++)
        {
            var pt = path2[i];

            var pip =
                InternalClipper.PointInPolygon(pt, path1);
            switch (pip)
            {
                case PointInPolygonResult.IsInside:
                    ioCount--; break;
                case PointInPolygonResult.IsOutside:
                    ioCount++; break;
            }
            if (Math.Abs(ioCount) > 1) break;
        }
        
        return ioCount <= 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsClockwise(Location prev, Location curr,
        PointD prevPt, PointD currPt, PointD rectMidPoint)
    {
        if (AreOpposites(prev, curr))
            return Clipper.LessThan(InternalClipper.CrossProduct(prevPt, rectMidPoint, currPt),0);
        return HeadingClockwise(prev, curr);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool AreOpposites(Location prev, Location curr)
    {
        return Math.Abs((int)prev - (int)curr) == 2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HeadingClockwise(Location prev, Location curr)
    {
        return ((int)prev + 1) % 4 == (int)curr;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Location GetAdjacentLocation(Location loc, bool isClockwise)
    {
        var delta = (isClockwise) ? 1 : 3;
        return (Location)(((int)loc + delta) % 4);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutPt2? UnlinkOp(OutPt2 op)
    {
        if (op.next == op) return null;
        op.prev!.next = op.next;
        op.next!.prev = op.prev;
        return op.next;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutPt2? UnlinkOpBack(OutPt2 op)
    {
        if (op.next == op) return null;
        op.prev!.next = op.next;
        op.next!.prev = op.prev;
        return op.prev;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static uint GetEdgesForPt(PointD pt, RectD rec)
    {
        uint result = 0;
        if (Clipper.AlmostEqual(pt.X, rec.Left)) result = 1;
        else if (Clipper.AlmostEqual(pt.X, rec.Right)) result = 4;
        if (Clipper.AlmostEqual(pt.Y, rec.Top)) result += 2;
        else if (Clipper.AlmostEqual(pt.Y, rec.Bottom)) result += 8;
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsHeadingClockwise(PointD pt1, PointD pt2, int edgeIdx)
    {
        return edgeIdx switch
        {
            0 => Clipper.LessThan(pt2.Y, pt1.Y),
            1 => Clipper.GreaterThan(pt2.X, pt1.X),
            2 => Clipper.GreaterThan(pt2.Y, pt1.Y),
            _ => Clipper.LessThan(pt2.X, pt1.X)
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasHorzOverlap(PointD left1, PointD right1,
        PointD left2, PointD right2)
    {
        return Clipper.LessThan(left1.X,right2.X) && Clipper.GreaterThan(right1.X, left2.X);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasVertOverlap(PointD top1, PointD bottom1,
        PointD top2, PointD bottom2)
    {
        return Clipper.LessThan(top1.Y, bottom2.Y) && Clipper.GreaterThan(bottom1.Y, top2.Y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddToEdge(List<OutPt2?> edge, OutPt2 op)
    {
        if (op.edge != null) return;
        op.edge = edge;
        edge.Add(op);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void UncoupleEdge(OutPt2 op)
    {
        if (op.edge == null) return;
        for (int i = 0; i < op.edge.Count; i++)
        {
            var op2 = op.edge[i];
            if (op2 != op) continue;
            op.edge[i] = null;
            break;
        }
        op.edge = null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetNewOwner(OutPt2 op, int newIdx)
    {
        op.ownerIdx = newIdx;
        var op2 = op.next!;
        while (op2 != op)
        {
            op2.ownerIdx = newIdx;
            op2 = op2.next!;
        }
    }

    private void AddCorner(Location prev, Location curr)
    {
        Add(HeadingClockwise(prev, curr) ? rectPath_[(int)prev] : rectPath_[(int)curr]);
    }

    private void AddCorner(ref Location loc, bool isClockwise)
    {
        if (isClockwise)
        {
            Add(rectPath_[(int)loc]);
            loc = GetAdjacentLocation(loc, true);
        }
        else
        {
            loc = GetAdjacentLocation(loc, false);
            Add(rectPath_[(int)loc]);
        }
    }

    static protected bool GetLocation(RectD rec, PointD pt, out Location loc)
    {
        if (Clipper.AlmostEqual(pt.X, rec.Left) && Clipper.GreaterThan(pt.Y, rec.Top) && Clipper.LessThan(pt.Y, rec.Bottom))
        {
            loc = Location.left; return false; 
        }
        if (Clipper.AlmostEqual(pt.X, rec.Right) && Clipper.GreaterThan(pt.Y, rec.Top) && Clipper.LessThan(pt.Y, rec.Bottom))
        {
            loc = Location.right; return false; 
        }
        if (Clipper.AlmostEqual(pt.Y, rec.Top) && Clipper.GreaterThan(pt.X, rec.Top) && Clipper.LessThan(pt.X, rec.Bottom))
        {
            loc = Location.top; return false; 
        }
        if (Clipper.AlmostEqual(pt.Y, rec.Right) && Clipper.GreaterThan(pt.X, rec.Top) && Clipper.LessThan(pt.X, rec.Bottom))
        {
            loc = Location.bottom; return false; 
        }
        if (Clipper.LessThanOrEqual(pt.X, rec.Left)) loc = Location.left;
        else if (Clipper.GreaterThanOrEqual(pt.X, rec.Right)) loc = Location.right;
        else if (Clipper.LessThanOrEqual(pt.Y, rec.Top)) loc = Location.top;
        else if (Clipper.GreaterThanOrEqual(pt.Y, rec.Bottom)) loc = Location.bottom;
        else loc = Location.inside;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsHorizontal(PointD pt1, PointD pt2)
    {
        return Clipper.AlmostEqual(pt1.Y,pt2.Y);
    }

    private static bool GetSegmentIntersection(PointD p1,
        PointD p2, PointD p3, PointD p4, out PointD ip)
    {
        var res1 = InternalClipper.CrossProduct(p1, p3, p4);
        var res2 = InternalClipper.CrossProduct(p2, p3, p4);
        if (Clipper.AlmostEqual(res1, 0))
        {
            ip = p1;
            if (Clipper.AlmostEqual(res2, 0)) return false; 
            if (p1 == p3 || p1 == p4) return true;
            if (IsHorizontal(p3, p4)) return ((Clipper.GreaterThan(p1.X, p3.X)) == Clipper.LessThan(p1.X, p4.X));
            return ((Clipper.GreaterThan(p1.Y, p3.Y) == Clipper.LessThan(p1.Y, p4.Y)));
        }
        if (Clipper.AlmostEqual(res2, 0))
        {
            ip = p2;
            if (p2 == p3 || p2 == p4) return true;
            if (IsHorizontal(p3, p4)) return ((Clipper.GreaterThan(p2.X, p3.X) == Clipper.LessThan(p2.X, p4.X)));
            return (Clipper.GreaterThan(p2.Y, p3.Y) == Clipper.LessThan(p2.Y, p4.Y));
        }

        if (Clipper.GreaterThan(res1, 0) == Clipper.GreaterThan(res2, 0))
        {
            ip = new PointD(0, 0);
            return false;
        }

        var res3 = InternalClipper.CrossProduct(p3, p1, p2);
        var res4 = InternalClipper.CrossProduct(p4, p1, p2);
        if (Clipper.AlmostEqual(res3, 0))
        {
            ip = p3;
            if (p3 == p1 || p3 == p2) return true;
            if (IsHorizontal(p1, p2)) return (Clipper.GreaterThan(p3.X, p1.X) == Clipper.LessThan(p3.X,p2.X));
            return (Clipper.GreaterThan(p3.Y, p1.Y) == Clipper.LessThan(p3.Y, p2.Y));
        }
        if (Clipper.AlmostEqual(res4, 0))
        {
            ip = p4;
            if (p4 == p1 || p4 == p2) return true;
            if (IsHorizontal(p1, p2)) return (Clipper.GreaterThan(p4.X, p1.X) == Clipper.LessThan(p4.X, p2.X));
            return (Clipper.GreaterThan(p4.Y, p1.Y) == Clipper.LessThan(p4.Y, p2.Y));
        }
        if (Clipper.GreaterThan(res3, 0) == Clipper.GreaterThan(res4, 0))
        {
            ip = new PointD(0, 0);
            return false;
        }

        return InternalClipper.GetSegmentIntersectPt(p1, p2, p3, p4, out ip);
    }

    static protected bool GetIntersection(PathD rectPath, PointD p, PointD p2, ref Location loc, out PointD ip)
    {
        ip = new PointD();
        switch (loc)
        {
            case Location.left:
                if (GetSegmentIntersection(p, p2, rectPath[0], rectPath[3], out ip))
                    return true;
                if (Clipper.LessThan(p.Y, rectPath[0].Y) && GetSegmentIntersection(p, p2, rectPath[0], rectPath[1], out ip))
                {
                    loc = Location.top;
                    return true;
                }

                if (!GetSegmentIntersection(p, p2, rectPath[2], rectPath[3], out ip)) return false;
                loc = Location.bottom;
                return true;

            case Location.right:
                if (GetSegmentIntersection(p, p2, rectPath[1], rectPath[2], out ip))
                    return true;
                if (Clipper.LessThan(p.Y, rectPath[0].Y) && GetSegmentIntersection(p, p2, rectPath[0], rectPath[1], out ip))
                {
                    loc = Location.top;
                    return true;
                }

                if (!GetSegmentIntersection(p, p2, rectPath[2], rectPath[3], out ip)) return false;
                loc = Location.bottom;
                return true;

            case Location.top:
                if (GetSegmentIntersection(p, p2, rectPath[0], rectPath[1], out ip))
                    return true;
                if (Clipper.LessThan(p.X,rectPath[0].X) && GetSegmentIntersection(p, p2, rectPath[0], rectPath[3], out ip))
                {
                    loc = Location.left;
                    return true;
                }

                if (Clipper.LessThanOrEqual(p.X, rectPath[1].X) || !GetSegmentIntersection(p, p2, rectPath[1], rectPath[2], out ip)) return false;
                loc = Location.right;
                return true;

            case Location.bottom:
                if (GetSegmentIntersection(p, p2, rectPath[2], rectPath[3], out ip))
                    return true;
                if (Clipper.LessThan(p.X, rectPath[3].X) && GetSegmentIntersection(p, p2, rectPath[0], rectPath[3], out ip))
                {
                    loc = Location.left;
                    return true;
                }

                if (Clipper.LessThanOrEqual(p.X, rectPath[2].X) || !GetSegmentIntersection(p, p2, rectPath[1], rectPath[2], out ip)) return false;
                loc = Location.right;
                return true;

            default:
                if (GetSegmentIntersection(p, p2, rectPath[0], rectPath[3], out ip))
                {
                    loc = Location.left;
                    return true;
                }
                if (GetSegmentIntersection(p, p2, rectPath[0], rectPath[1], out ip))
                {
                    loc = Location.top;
                    return true;
                }
                if (GetSegmentIntersection(p, p2, rectPath[1], rectPath[2], out ip))
                {
                    loc = Location.right;
                    return true;
                }

                if (!GetSegmentIntersection(p, p2, rectPath[2], rectPath[3], out ip)) return false;
                loc = Location.bottom;
                return true;
        }
    }

    protected void GetNextLocation(PathD path,
      ref Location loc, ref int i, int highI)
    {
        switch (loc)
        {
            case Location.left:
                {
                    while (Clipper.LessThanOrEqual(i,highI) && Clipper.LessThanOrEqual(path[i].X, rect_.Left)) i++;
                    if (Clipper.GreaterThan(i, highI)) break;
                    if (Clipper.GreaterThanOrEqual(path[i].X, rect_.Right)) loc = Location.right;
                    else if (Clipper.LessThanOrEqual(path[i].Y, rect_.Top)) loc = Location.top;
                    else if (Clipper.GreaterThanOrEqual(path[i].Y, rect_.Bottom)) loc = Location.bottom;
                    else loc = Location.inside;
                }
                break;

            case Location.top:
                {
                    while (Clipper.LessThanOrEqual(i, highI) && Clipper.LessThanOrEqual(path[i].Y, rect_.Top)) i++;
                    if (Clipper.GreaterThan(i, highI)) break;
                    if (Clipper.GreaterThanOrEqual(path[i].Y, rect_.Bottom)) loc = Location.bottom;
                    else if (Clipper.LessThanOrEqual(path[i].X, rect_.Left)) loc = Location.left;
                    else if (Clipper.GreaterThanOrEqual(path[i].X, rect_.Right)) loc = Location.right;
                    else loc = Location.inside;
                }
                break;

            case Location.right:
                {
                    while (Clipper.LessThanOrEqual(i,highI) && Clipper.LessThanOrEqual(path[i].X, rect_.Right)) i++;
                    if (Clipper.GreaterThan(i, highI)) break;
                    if (Clipper.LessThanOrEqual(path[i].X, rect_.Left)) loc = Location.left;
                    else if (Clipper.LessThanOrEqual(path[i].Y, rect_.Top)) loc = Location.top;
                    else if (Clipper.GreaterThanOrEqual(path[i].Y, rect_.Bottom)) loc = Location.bottom;
                    else loc = Location.inside;
                }
                break;

            case Location.bottom:
                {
                    while (Clipper.LessThanOrEqual(i, highI) && Clipper.GreaterThanOrEqual(path[i].Y, rect_.Bottom)) i++;
                    if (Clipper.GreaterThan(i, highI)) break;
                    if (Clipper.LessThanOrEqual(path[i].Y, rect_.Top)) loc = Location.top;
                    else if (Clipper.LessThanOrEqual(path[i].X, rect_.Left)) loc = Location.left;
                    else if (Clipper.GreaterThanOrEqual(path[i].X, rect_.Right)) loc = Location.right;
                    else loc = Location.inside;
                }
                break;

            case Location.inside:
                {
                    while (i <= highI)
                    {
                        if (Clipper.LessThan(path[i].X, rect_.Left)) loc = Location.left;
                        else if (Clipper.GreaterThan(path[i].X,rect_.Right)) loc = Location.right;
                        else if (Clipper.GreaterThan(path[i].Y, rect_.Bottom)) loc = Location.bottom;
                        else if (Clipper.LessThan(path[i].Y, rect_.Top)) loc = Location.top;
                        else
                        {
                            Add(path[i]);
                            i++;
                            continue;
                        }
                        break;
                    }
                }
                break;
        } 
    }

    private static bool StartLocsAreClockwise(List<Location> startLocs)
    {
        var result = 0;
        for (var i = 1; i < startLocs.Count; i++)
        {
            var d = (int)startLocs[i] - (int)startLocs[i - 1];
            switch (d)
            {
                case -1: result -= 1; break;
                case 1: result += 1; break;
                case -3: result += 1; break;
                case 3: result -= 1; break;
            }
        }
        return result > 0;
    }

    private void ExecuteInternal(PathD path)
    {
        if (path.Count < 3 || rect_.IsEmpty()) return;
        var startLocs = new List<Location>();

        var firstCross = Location.inside;
        Location crossingLoc = firstCross, prev = firstCross;

        int i, highI = path.Count - 1;
        if (!GetLocation(rect_, path[highI], out var loc))
        {
            i = highI - 1;
            while (i >= 0 && !GetLocation(rect_, path[i], out prev)) i--;
            if (i < 0)
            {
                for (var j = 0; j < path.Count; j++)
                {
                    Add(path[j]);
                }
                return;
            }
            if (prev == Location.inside) loc = Location.inside;
        }
        var startingLoc = loc;

        i = 0;
        while (i <= highI)
        {
            prev = loc;
            Location prevCrossLoc = crossingLoc;
            GetNextLocation(path, ref loc, ref i, highI);
            if (i > highI) break;

            var prevPt = (i == 0) ? path[highI] : path[i - 1];
            crossingLoc = loc;
            if (!GetIntersection(rectPath_,
              path[i], prevPt, ref crossingLoc, out var ip))
            {
                if (prevCrossLoc == Location.inside)
                {
                    var isClockw = IsClockwise(prev, loc, prevPt, path[i], mp_);
                    do
                    {
                        startLocs.Add(prev);
                        prev = GetAdjacentLocation(prev, isClockw);
                    } while (prev != loc);
                    crossingLoc = prevCrossLoc; 
                }

                else if (prev != Location.inside && prev != loc)
                {
                    var isClockw = IsClockwise(prev, loc, prevPt, path[i], mp_);
                    do
                    {
                        AddCorner(ref prev, isClockw);
                    } while (prev != loc);
                }
                ++i;
                continue;
            }

            if (loc == Location.inside)
            {
                if (firstCross == Location.inside)
                {
                    firstCross = crossingLoc;
                    startLocs.Add(prev);
                }
                else if (prev != crossingLoc)
                {
                    var isClockw = IsClockwise(prev, crossingLoc, prevPt, path[i], mp_);
                    do
                    {
                        AddCorner(ref prev, isClockw);
                    } while (prev != crossingLoc);
                }
            }
            else if (prev != Location.inside)
            {
                loc = prev;
                GetIntersection(rectPath_,
                  prevPt, path[i], ref loc, out var ip2);
                if (prevCrossLoc != Location.inside && prevCrossLoc != loc) 
                    AddCorner(prevCrossLoc, loc);

                if (firstCross == Location.inside)
                {
                    firstCross = loc;
                    startLocs.Add(prev);
                }

                loc = crossingLoc;
                Add(ip2);
                if (ip == ip2)
                {
                    GetLocation(rect_, path[i], out loc);
                    AddCorner(crossingLoc, loc);
                    crossingLoc = loc;
                    continue;
                }
            }
            else 
            {
                loc = crossingLoc;
                if (firstCross == Location.inside)
                    firstCross = crossingLoc;
            }

            Add(ip);
        }

        if (firstCross == Location.inside)
        {
            if (startingLoc == Location.inside) return;
            if (!pathBounds_.Contains(rect_) ||
                !Path1ContainsPath2(path, rectPath_)) return;
            var startLocsClockwise = StartLocsAreClockwise(startLocs);
            for (var j = 0; j < 4; j++)
            {
                var k = startLocsClockwise ? j : 3 - j; 
                Add(rectPath_[k]);
                AddToEdge(edges_[k * 2], results_[0]!);
            }
        }
        else if (loc != Location.inside &&
          (loc != firstCross || startLocs.Count > 2))
        {
            if (startLocs.Count > 0)
            {
                prev = loc;

                for (var j = 0; j < startLocs.Count; j++)
                {
                    var loc2 = startLocs[j];
                    if (prev == loc2) continue;
                    AddCorner(ref prev, HeadingClockwise(prev, loc2));
                    prev = loc2;
                }

                loc = prev;
            }
            if (loc != firstCross)
                AddCorner(ref loc, HeadingClockwise(loc, firstCross));
        }
    }

    public PathsD Execute(PathsD paths)
    {
        var result = new PathsD();
        if (rect_.IsEmpty()) return result;

        for (var j = 0; j < paths.Count; j++)
        {
            var path = paths[j];

            if (path.Count < 3) continue;
            pathBounds_ = Clipper.GetBounds(path);
            if (!rect_.Intersects(pathBounds_))
                continue; 
            if (rect_.Contains(pathBounds_))
            {
                result.Add(path);
                continue;
            }
            ExecuteInternal(path);
            CheckEdges();
            for (var i = 0; i < 4; ++i)
                TidyEdgePair(i, edges_[i * 2], edges_[i * 2 + 1]);

            for (var i = 0; i < results_.Count; i++)
            {
                var op = results_[i];
                PathD tmp = GetPath(op);
                if (tmp.Count > 0) result.Add(tmp);
            }

            results_.Clear();
            for (int i = 0; i < 8; i++)
                edges_[i].Clear();
        }
        return result;
    }

    private void CheckEdges()
    {
        for (var i = 0; i < results_.Count; i++)
        {
            OutPt2? op = results_[i], op2 = op;
            if (op == null) continue;
            do
            {
                if (InternalClipper.IsCollinear(
                        op2!.prev!.pt, op2.pt, op2.next!.pt))
                {
                    if (op2 == op)
                    {
                        op2 = UnlinkOpBack(op2);
                        if (op2 == null) break;
                        op = op2.prev;
                    }
                    else
                    {
                        op2 = UnlinkOpBack(op2);
                        if (op2 == null) break;
                    }
                }
                else
                    op2 = op2.next;
            } while (op2 != op);

            if (op2 == null)
            {
                results_[i] = null;
                continue;
            }
            results_[i] = op2; 

            var edgeSet1 = GetEdgesForPt(op!.prev!.pt, rect_);
            op2 = op;
            do
            {
                var edgeSet2 = GetEdgesForPt(op2!.pt, rect_);
                if (edgeSet2 != 0 && op2.edge == null)
                {
                    var combinedSet = (edgeSet1 & edgeSet2);
                    for (var j = 0; j < 4; ++j)
                    {
                        if ((combinedSet & (1 << j)) == 0) continue;
                        if (IsHeadingClockwise(op2.prev!.pt, op2.pt, j))
                            AddToEdge(edges_[j * 2], op2);
                        else
                            AddToEdge(edges_[j * 2 + 1], op2);
                    }
                }
                edgeSet1 = edgeSet2;
                op2 = op2.next;
            } while (op2 != op);
        }
    }

    private void TidyEdgePair(int idx, List<OutPt2?> cw, List<OutPt2?> ccw)
    {
        if (ccw.Count == 0) return;
        var isHorz = idx is 1 or 3;
        var cwIsTowardLarger = idx is 1 or 2;
        int i = 0, j = 0;

        while (i < cw.Count)
        {
            var p1 = cw[i];
            if (p1 == null || p1.next == p1.prev)
            {
                cw[i++] = null;
                j = 0;
                continue;
            }

            var jLim = ccw.Count;
            while (j < jLim &&
              (ccw[j] == null || ccw[j]!.next == ccw[j]!.prev)) ++j;

            if (j == jLim)
            {
                ++i;
                j = 0;
                continue;
            }

            OutPt2? p2;
            OutPt2? p1a;
            OutPt2? p2a;
            if (cwIsTowardLarger)
            {
                p1 = cw[i]!.prev!;
                p1a = cw[i];
                p2 = ccw[j];
                p2a = ccw[j]!.prev!;
            }
            else
            {
                p1 = cw[i];
                p1a = cw[i]!.prev!;
                p2 = ccw[j]!.prev!;
                p2a = ccw[j];
            }

            if ((isHorz && !HasHorzOverlap(p1!.pt, p1a!.pt, p2!.pt, p2a!.pt)) ||
              (!isHorz && !HasVertOverlap(p1!.pt, p1a!.pt, p2!.pt, p2a!.pt)))
            {
                ++j;
                continue;
            }

            var isRejoining = cw[i]!.ownerIdx != ccw[j]!.ownerIdx;

            if (isRejoining)
            {
                results_[p2!.ownerIdx] = null;
                SetNewOwner(p2, p1!.ownerIdx);
            }

            if (cwIsTowardLarger)
            {
                p1!.next = p2;
                p2!.prev = p1;
                p1a!.prev = p2a;
                p2a!.next = p1a;
            }
            else
            {
                p1!.prev = p2;
                p2!.next = p1;
                p1a!.next = p2a;
                p2a!.prev = p1a;
            }

            if (!isRejoining)
            {
                var new_idx = results_.Count;
                results_.Add(p1a);
                SetNewOwner(p1a, new_idx);
            }

            OutPt2? op;
            OutPt2? op2;
            if (cwIsTowardLarger)
            {
                op = p2;
                op2 = p1a;
            }
            else
            {
                op = p1;
                op2 = p2a;
            }
            results_[op.ownerIdx] = op;
            results_[op2.ownerIdx] = op2;


            bool opIsLarger, op2IsLarger;
            if (isHorz)
            {
                opIsLarger = Clipper.GreaterThan(op.pt.X, op.prev!.pt.X);
                op2IsLarger = Clipper.GreaterThan(op2.pt.X, op2.prev!.pt.X);
            }
            else      
            {
                opIsLarger = Clipper.GreaterThan(op.pt.Y, op.prev!.pt.Y);
                op2IsLarger = Clipper.GreaterThan(op2.pt.Y, op2.prev!.pt.Y);
            }

            if ((op.next == op.prev) ||
              (op.pt == op.prev.pt))
            {
                if (op2IsLarger == cwIsTowardLarger)
                {
                    cw[i] = op2;
                    ccw[j++] = null;
                }
                else
                {
                    ccw[j] = op2;
                    cw[i++] = null;
                }
            }
            else if ((op2.next == op2.prev) ||
              (op2.pt == op2.prev.pt))
            {
                if (opIsLarger == cwIsTowardLarger)
                {
                    cw[i] = op;
                    ccw[j++] = null;
                }
                else
                {
                    ccw[j] = op;
                    cw[i++] = null;
                }
            }
            else if (opIsLarger == op2IsLarger)
            {
                if (opIsLarger == cwIsTowardLarger)
                {
                    cw[i] = op;
                    UncoupleEdge(op2);
                    AddToEdge(cw, op2);
                    ccw[j++] = null;
                }
                else
                {
                    cw[i++] = null;
                    ccw[j] = op2;
                    UncoupleEdge(op);
                    AddToEdge(ccw, op);
                    j = 0;
                }
            }
            else
            {
                if (opIsLarger == cwIsTowardLarger)
                    cw[i] = op;
                else
                    ccw[j] = op;
                if (op2IsLarger == cwIsTowardLarger)
                    cw[i] = op2;
                else
                    ccw[j] = op2;
            }
        }
    }

    private static PathD GetPath(OutPt2? op)
    {
        var result = new PathD();
        if (op == null || op.prev == op.next) return result;
        var op2 = op.next;
        while (op2 != null && op2 != op)
        {
            if (InternalClipper.IsCollinear(
                    op2.prev!.pt, op2.pt, op2.next!.pt))
            {
                op = op2.prev;
                op2 = UnlinkOp(op2);
            }
            else
                op2 = op2.next;
        }
        if (op2 == null) return new PathD();

        result.Add(op.pt);
        op2 = op.next;
        while (op2 != op)
        {
            result.Add(op2!.pt);
            op2 = op2.next;
        }
        return result;
    }

} 


