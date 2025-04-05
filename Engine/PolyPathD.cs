using ExtensionClipper2.Core;
using System.Runtime.CompilerServices;

namespace ExtensionClipper2.Engine;

public class PolyPathD : PolyPathBase
{
    public PathD? Polygon { get; private set; }

    public PolyPathD(PolyPathBase? parent = null) : base(parent) { }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override PolyPathBase AddChild(PathD p)
    {
        PolyPathBase newChild = new PolyPathD(this);
        (newChild as PolyPathD)!.Polygon = p;
        _childs.Add(newChild);
        return newChild;
    }

    [IndexerName("Child")]
    public PolyPathD this[int index]
    {
        get
        {
            if (index < 0 || index >= _childs.Count)
                throw new InvalidOperationException();
            return (PolyPathD)_childs[index];
        }
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public double Area()
    {
        var result = Polygon == null ? 0 : Clipper.Area(Polygon);

        for (var i = 0; i < _childs.Count; i++)
        {
            var child = (PolyPathD)_childs[i];
            result += child.Area();
        }
        return result;
    }
}

public class PolyTreeD : PolyPathD
{
    
}

