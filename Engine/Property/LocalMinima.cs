using ExtensionClipper2.Enums;

namespace ExtensionClipper2.Engine.Property;

internal readonly struct LocalMinima
{
    public readonly Vertex vertex;
    public readonly PathType polytype;
    public readonly bool isOpen;

    public LocalMinima(Vertex vertex, PathType polytype, bool isOpen = false)
    {
        this.vertex = vertex;
        this.polytype = polytype;
        this.isOpen = isOpen;
    }

    public static bool operator ==(LocalMinima lm1, LocalMinima lm2)
    {
        return ReferenceEquals(lm1.vertex, lm2.vertex);
    }

    public static bool operator !=(LocalMinima lm1, LocalMinima lm2)
    {
        return !(lm1 == lm2);
    }

    public override bool Equals(object? obj)
    {
        return obj is LocalMinima minima && this == minima;
    }

    public override int GetHashCode()
    {
        return vertex.GetHashCode();
    }
}

internal struct LocMinSorter : IComparer<LocalMinima>
{
    public readonly int Compare(LocalMinima locMin1, LocalMinima locMin2)
    {
        return locMin2.vertex.pt.Y.CompareTo(locMin1.vertex.pt.Y);
    }
}

