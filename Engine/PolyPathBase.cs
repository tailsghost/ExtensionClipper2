using System.Collections;
using System.Runtime.CompilerServices;
using ExtensionClipper2.Core;

namespace ExtensionClipper2.Engine;

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

    public bool IsHole => GetIsHole();

    public PolyPathBase(PolyPathBase? parent = null) { _parent = parent; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int GetLevel()
    {
        var result = 0;
        var pp = _parent;
        while (pp != null) { ++result; pp = pp._parent; }
        return result;
    }

    public int Level => GetLevel();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool GetIsHole()
    {
        int lvl = GetLevel();
        return lvl != 0 && (lvl & 1) == 0;
    }

    public int Count => _childs.Count;
    public abstract PolyPathBase AddChild(PathD p);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Clear()
    {
        _childs.Clear();
    }

    internal string ToStringInternal(int idx, int level)
    {
        string result = "", padding = "", plural = "s";
        if (_childs.Count == 1) plural = "";
        padding = padding.PadLeft(level * 2);
        if ((level & 1) == 0)
            result += $"{padding}+- hole ({idx}) contains {_childs.Count} nested polygon{plural}.\n";
        else
            result += $"{padding}+- polygon ({idx}) contains {_childs.Count} hole{plural}.\n";

        for (var i = 0; i < Count; i++)
            if (_childs[i].Count > 0)
                result += _childs[i].ToStringInternal(i, level + 1);
        return result;
    }

    public override string ToString()
    {
        if (Level > 0) return "";
        string plural = "s";
        if (_childs.Count == 1) plural = "";
        string result = $"Polytree with {_childs.Count} polygon{plural}.\n";
        for (int i = 0; i < Count; i++)
            if (_childs[i].Count > 0)
                result += _childs[i].ToStringInternal(i, 1);
        return result + '\n';
    }

}

