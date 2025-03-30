using ExtensionClipper2.Core;

namespace ExtensionClipper2.RectClips.Property;

public class OutPt2
{
    public OutPt2? next;
    public OutPt2? prev;

    public PointD pt;
    public int ownerIdx;
    public List<OutPt2?>? edge;
    public OutPt2(PointD pt)
    {
        this.pt = pt;
    }
}

