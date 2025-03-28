namespace ExtensionClipper2.Enums;

[Flags]
public enum PointInPolygonResult
{
    IsOn = 0,
    IsInside = 1,
    IsOutside = 2
}

[Flags]
internal enum VertexFlags
{
    None = 0,
    OpenStart = 1,
    OpenEnd = 2,
    LocalMax = 4,
    LocalMin = 8
}

