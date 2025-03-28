namespace ExtensionClipper2.Engine.Property;

internal class HorzSegment
{
    public OutPt? leftOp;
    public OutPt? rightOp;
    public bool leftToRight;
    public HorzSegment(OutPt op)
    {
        leftOp = op;
        rightOp = null;
        leftToRight = true;
    }
}

