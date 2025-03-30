using ExtensionClipper2.Core;
using ExtensionClipper2.Enums;

namespace ExtensionClipper2.Engine.Property;

public class Vertex
{
    public readonly PointD pt;
    public Vertex? next;
    public Vertex? prev;
    public VertexFlags flags;

    public Vertex(PointD pt, VertexFlags flags, Vertex? prev)
    {
        this.pt = pt;
        this.flags = flags;
        next = null;
        this.prev = prev;
    }
}

