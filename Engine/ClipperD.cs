using System.Collections;
using ExtensionClipper2.Core;
using ExtensionClipper2.Enums;
using System.IO;
using System.Runtime.CompilerServices;

namespace ExtensionClipper2.Engine;

public class ClipperD : ClipperBase
{
    private const string precision_range_error = "Error: Precision is out of range.";
    private readonly double _scale;
    private readonly double _invScale;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddPath(PathD path, PathType polytype, bool isOpen = false)
    {
        base.AddPath(path, polytype, isOpen);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddPaths(PathsD paths, PathType polytype, bool isOpen = false)
    {
        base.AddPaths(paths, polytype, isOpen);
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
    public void AddSubject(PathsD paths)
    {
        AddPaths(paths, PathType.Subject);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddOpenSubject(PathsD paths)
    {
        AddPaths(paths, PathType.Subject, true);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddClip(PathsD paths)
    {
        AddPaths(paths, PathType.Clip);
    }

    public bool Execute(ClipType clipType, FillRule fillRule,
        PathsD solutionClosed, PathsD solutionOpen)
    {
        PathsD solClosed = new PathsD(), solOpen = new PathsD();

        var success = true;
        solutionClosed.Clear();
        solutionOpen.Clear();
        try
        {
            ExecuteInternal(clipType, fillRule);
            BuildPaths(solClosed, solOpen);
        }
        catch
        {
            success = false;
        }

        ClearSolutionOnly();
        if (!success) return false;

        solutionClosed.EnsureCapacity(solClosed.Count);

        for (var i = 0; i < solClosed.Count; i++)
        {
            solutionClosed.Add(solClosed[i]);
        }

        solutionOpen.EnsureCapacity(solOpen.Count);
        for (var i = 0; i < solOpen.Count; i++)
        {
            solutionOpen.Add(solOpen[i]);
        }

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Execute(ClipType clipType, FillRule fillRule, PathsD solutionClosed)
    {
        return Execute(clipType, fillRule, solutionClosed, new PathsD());
    }

    public bool Execute(ClipType clipType, FillRule fillRule, PolyTreeD polytree, PathsD openPaths)
    {
        polytree.Clear();
        openPaths.Clear();
        _using_polytree = true;
        (polytree as PolyPathD).Scale = _scale;

        var oPaths = new PathsD();
        var success = true;
        try
        {
            ExecuteInternal(clipType, fillRule);
            BuildTree(polytree, oPaths);
        }
        catch
        {
            success = false;
        }

        ClearSolutionOnly();
        if (!success) return false;
        if (oPaths.Count <= 0) return true;
        openPaths.EnsureCapacity(oPaths.Count);
        foreach (PathD path in oPaths)
            openPaths.Add(path);

        return true;
    }

    public bool Execute(ClipType clipType, FillRule fillRule, PolyTreeD polytree)
    {
        return Execute(clipType, fillRule, polytree, new PathsD());
    }

    public abstract class PolyPathBase : IEnumerable
    {
        internal PolyPathBase? _parent;
        internal List<PolyPathBase> _childs = new List<PolyPathBase>();

        public IEnumerator GetEnumerator()
        {
            return new NodeEnumerator(_childs);
        }

        private class NodeEnumerator : IEnumerator
        {
            private int position = -1;
            private readonly List<PolyPathBase> _nodes;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public NodeEnumerator(List<PolyPathBase> nodes)
            {
                _nodes = new List<PolyPathBase>(nodes);
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool MoveNext()
            {
                position++;
                return (position < _nodes.Count);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Reset()
            {
                position = -1;
            }

            public object Current
            {
                get
                {
                    if (position < 0 || position >= _nodes.Count)
                        throw new InvalidOperationException();
                    return _nodes[position];
                }
            }

        }
    }
}

