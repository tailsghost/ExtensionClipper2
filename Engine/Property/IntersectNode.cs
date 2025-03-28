using System;
using ExtensionClipper2.Core;

namespace ExtensionClipper2.Engine.Property;

internal readonly struct IntersectNode
{
    public readonly PointD pt;
    public readonly Active edge1;
    public readonly Active edge2;

    public IntersectNode(PointD pt, Active edge1, Active edge2)
    {
        this.pt = pt;
        this.edge1 = edge1;
        this.edge2 = edge2;
    }
}

