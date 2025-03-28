using ExtensionClipper2.Core;

namespace ExtensionClipper2.Engine.Property;

internal class OutPt
{
    public PointD pt;
    public OutPt? next;
    public OutPt prev;
    public OutRec outrec;
    public HorzSegment? horz;

    public OutPt(PointD pt, OutRec outrec)
    {
        this.pt = pt;
        this.outrec = outrec;
        next = this;
        prev = this;
        horz = null;
    }
}

