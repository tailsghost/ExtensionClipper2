using ExtensionClipper2.Core;
using System;

namespace ExtensionClipper2.Engine.Property;

internal class OutRec
{
    public int idx;
    public OutRec? owner;
    public Active? frontEdge;
    public Active? backEdge;
    public OutPt? pts;
    public PolyPathBase? polypath;
    public RectD bounds;
    public PathD path = new();
    public bool isOpen;
    public List<int>? splits;
    public OutRec? recursiveSplit;
}

