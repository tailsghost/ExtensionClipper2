namespace ExtensionClipper2.Core;

public class PathD : List<PointD>
{
    public PathD() : base() { }
    public PathD(int capacity = 0) : base(capacity) { }
    public PathD(IEnumerable<PointD> path) : base(path) { }
    public string ToString(int precision = 2)
    {
        var s = "";

        for (var i = 0; i < this.Count; i++)
            s = s + this[i].ToString(precision) + ", ";

        if (s != "") s = s.Remove(s.Length - 2);
        return s;
    }
}

public class PathsD : List<PathD>
{
    public PathsD() : base() { }
    public PathsD(int capacity = 0) : base(capacity) { }
    public PathsD(IEnumerable<PathD> paths) : base(paths) { }
    public string ToString(int precision = 2)
    {
        string s = "";
        for (int i = 0; i < this.Count; i++)
        {
            s = s + this[i].ToString(precision) + "\n";
        }

        return s;
    }
}

