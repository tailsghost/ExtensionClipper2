using ExtensionClipper2.Core;
using ExtensionClipper2.Enums;

namespace ExtensionClipper2.Engine.Property;

public class ReuseableDataContainer
{
    internal readonly List<LocalMinima> _minimaList;
    internal readonly List<Vertex> _vertexList;
    public ReuseableDataContainer()
    {
        _minimaList = new List<LocalMinima>();
        _vertexList = new List<Vertex>();
    }
    public void Clear()
    {
        _minimaList.Clear();
        _vertexList.Clear();
    }

    public void AddPaths(PathsD paths, PathType pt, bool isOpen)
    {
        ClipperEngine.AddPathsToVertexList(paths, pt, isOpen, _minimaList, _vertexList);
    }
}

