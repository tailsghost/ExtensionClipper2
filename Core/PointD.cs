namespace ExtensionClipper2.Core;

public struct PointD
{
    public double X;
    public double Y;

    public PointD(PointD pt)
    {
        X = pt.X; Y = pt.Y;
    }

    public PointD(PointD pt, double scale)
    {
        X = pt.X * scale;
        Y = pt.Y * scale;
    }

    public PointD(double x, double y)
    {
        X = x; Y = y;
    }

    public readonly string ToString(int precision = 2)
    {
        return string.Format($"{{0:F{precision}}, {{1:F{precision}}}", X, Y);
    }

    public static bool operator ==(PointD lhs, PointD rhs)
    {
        return InternalClipper.IsAlmostZero(lhs.x - rhs.x) &&
               InternalClipper.IsAlmostZero(lhs.y - rhs.y);
    }

    public static bool operator !=(PointD lhs, PointD rhs)
    {
        return !InternalClipper.IsAlmostZero(lhs.x - rhs.x) ||
               !InternalClipper.IsAlmostZero(lhs.y - rhs.y);
    }

    public readonly override bool Equals(object? obj)
    {
        if (obj != null && obj is PointD p)
            return this == p;
        return false;
    }

    public void Negate() { X = -X; Y = -Y; }

    public readonly override int GetHashCode()
    {
        return HashCode.Combine(X, Y); 
    }
}

