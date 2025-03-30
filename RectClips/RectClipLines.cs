using ExtensionClipper2.Core;
using ExtensionClipper2.Enums;
using ExtensionClipper2.RectClips.Property;
using System.IO;

namespace ExtensionClipper2.RectClips;

internal class RectClipLines : RectClip
{
    internal RectClipLines(RectD rect) : base(rect) { }

    public new PathsD Execute(PathsD paths)
    {
        var result = new PathsD();
        if (rect_.IsEmpty()) return result;


        for (var i = 0; i < paths.Count; i++)
        {
            var path = paths[i];

            if (path.Count < 2) continue;
            pathBounds_ = Clipper.GetBounds(path);
            if (!rect_.Intersects(pathBounds_))
                continue; 
            ExecuteInternal(path);

            for (var j = 0; j < results_.Count; j++)
            {
                var op = results_[j];
                var tmp = GetPath(op);
                if (tmp.Count > 0) result.Add(tmp);
            }

            results_.Clear();
            for (var j = 0; j < 8; j++)
                edges_[j].Clear();
        }
        return result;
    }

    private static PathD GetPath(OutPt2? op)
    {
        var result = new PathD();
        if (op == null || op == op.next) return result;
        op = op.next; 
        result.Add(op!.pt);
        var op2 = op.next!;
        while (op2 != op)
        {
            result.Add(op2.pt);
            op2 = op2.next!;
        }
        return result;
    }

    private void ExecuteInternal(PathD path)
    {
        results_.Clear();
        if (path.Count < 2 || rect_.IsEmpty()) return;

        Location prev = Location.inside;
        int i = 1, highI = path.Count - 1;
        if (!GetLocation(rect_, path[0], out var loc))
        {
            while (i <= highI && !GetLocation(rect_, path[i], out prev)) i++;
            if (i > highI)
            {
                for (var j = 0; j < path.Count; j++)
                    Add(path[j]);
                return;
            }
            if (prev == Location.inside) loc = Location.inside;
            i = 1;
        }
        if (loc == Location.inside) Add(path[0]);

        while (i <= highI)
        {
            prev = loc;
            GetNextLocation(path, ref loc, ref i, highI);
            if (i > highI) break;
            var prevPt = path[i - 1];

            var crossingLoc = loc;
            if (!GetIntersection(rectPath_, path[i], prevPt, ref crossingLoc, out var ip))
            {
                ++i;
                continue;
            }

            if (loc == Location.inside) 
            {
                Add(ip, true);
            }
            else if (prev != Location.inside)
            {
                crossingLoc = prev;
                GetIntersection(rectPath_, prevPt, path[i], ref crossingLoc, out var ip2);
                Add(ip2, true);
                Add(ip);
            }
            else
            {
                Add(ip);
            }
        }    
    } 

} 


