using ExtensionClipper2.Core;
using ExtensionClipper2.Engine.Property;
using ExtensionClipper2.Enums;
using System.Runtime.CompilerServices;

namespace ExtensionClipper2.Engine;

internal static class ClipperEngine
{

    internal static void AddLocMin(Vertex vert, PathType polytype, bool isOpen,
      List<LocalMinima> minimaList)
    {
        if ((vert.flags & VertexFlags.LocalMin) != VertexFlags.None) return;
        vert.flags |= VertexFlags.LocalMin;

        LocalMinima lm = new LocalMinima(vert, polytype, isOpen);
        minimaList.Add(lm);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void EnsureCapacity<T>(this List<T> list, int minCapacity)
    {
        if (list.Capacity < minCapacity)
            list.Capacity = minCapacity;
    }

    internal static void AddPathsToVertexList(PathsD paths, PathType polytype, bool isOpen,
      List<LocalMinima> minimaList, List<Vertex> vertexList)
    {
        var totalVertCnt = 0;

        for (var i = 0; i < paths.Count; i++)
        {
            totalVertCnt += paths[i].Count;
        }

        vertexList.EnsureCapacity(vertexList.Count + totalVertCnt);

        for (var i = 0; i < paths.Count; i++)
        {
            var path = paths[i];
            Vertex? v0 = null, prev_v = null, curr_v;
            for (var j = 0; j < path.Count; j++)
            {
                var pt = path[j];
                if (v0 == null)
                {
                    v0 = new Vertex(pt, VertexFlags.None, null);
                    vertexList.Add(v0);
                    prev_v = v0;
                }
                else if (prev_v!.pt != pt)
                {
                    curr_v = new Vertex(pt, VertexFlags.None, prev_v);
                    vertexList.Add(curr_v);
                    prev_v.next = curr_v;
                    prev_v = curr_v;
                }
            }
            if (prev_v?.prev == null) continue;
            if (!isOpen && Clipper.VertexValueEquals(prev_v, v0!)) prev_v = prev_v.prev;
            prev_v.next = v0;
            v0!.prev = prev_v;
            if (!isOpen && prev_v.next == prev_v) continue;

            bool going_up;
            if (isOpen)
            {
                curr_v = v0.next;
                while (curr_v != v0 && curr_v!.pt.Y == v0.pt.Y)
                    curr_v = curr_v.next;
                going_up = curr_v.pt.Y <= v0.pt.Y;
                if (going_up)
                {
                    v0.flags = VertexFlags.OpenStart;
                    AddLocMin(v0, polytype, true, minimaList);
                }
                else
                    v0.flags = VertexFlags.OpenStart | VertexFlags.LocalMax;
            }
            else
            {
                prev_v = v0.prev;
                while (prev_v != v0 && Clipper.AlmostEqual(prev_v!.pt.Y, v0.pt.Y))
                    prev_v = prev_v.prev;
                if (prev_v == v0)
                    continue;
                going_up = !Clipper.AlmostEqual(prev_v.pt.Y, v0.pt.Y) && prev_v.pt.Y > v0.pt.Y;
            }

            var going_up0 = going_up;
            prev_v = v0;
            curr_v = v0.next;
            while (curr_v != v0)
            {
                if (!Clipper.AlmostEqual(curr_v.pt.Y, prev_v.pt.Y) && curr_v!.pt.Y > prev_v.pt.Y && going_up)
                {
                    prev_v.flags |= VertexFlags.LocalMax;
                    going_up = false;
                }
                else if (!Clipper.AlmostEqual(curr_v.pt.Y, prev_v.pt.Y) && curr_v.pt.Y < prev_v.pt.Y && !going_up)
                {
                    going_up = true;
                    AddLocMin(prev_v, polytype, isOpen, minimaList);
                }
                prev_v = curr_v;
                curr_v = curr_v.next;
            }

            if (isOpen)
            {
                prev_v.flags |= VertexFlags.OpenEnd;
                if (going_up)
                    prev_v.flags |= VertexFlags.LocalMax;
                else
                    AddLocMin(prev_v, polytype, isOpen, minimaList);
            }
            else if (going_up != going_up0)
            {
                if (going_up0) AddLocMin(prev_v, polytype, false, minimaList);
                else prev_v.flags |= VertexFlags.LocalMax;
            }
        }
    }
}

