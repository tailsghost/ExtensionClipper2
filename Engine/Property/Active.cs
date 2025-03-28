using ExtensionClipper2.Core;
using ExtensionClipper2.Enums;

namespace ExtensionClipper2.Engine.Property;

internal class Active
{
    public PointD bot;
    public PointD top;
    public double curX; 
    public double dx;
    public int windDx; 
    public int windCount;
    public int windCount2; 
    public OutRec? outrec;

    public Active? prevInAEL;
    public Active? nextInAEL;

    public Active? prevInSEL;
    public Active? nextInSEL;
    public Active? jump;
    public Vertex? vertexTop;
    public LocalMinima localMin;
    internal bool isLeftBound;
    internal JoinWith joinWith;
}

