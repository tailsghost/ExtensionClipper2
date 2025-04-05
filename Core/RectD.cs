namespace ExtensionClipper2.Core;

public struct RectD
{
    public double Left;
    public double Top;
    public double Right;
    public double Bottom;

    public RectD(double l, double t, double r, double b)
    {
        Left = l;
        Top = t;
        Right = r;
        Bottom = b;
    }

    public RectD(RectD rec)
    {
        Left = rec.Left;
        Top = rec.Top;
        Right = rec.Right;
        Bottom = rec.Bottom;
    }

    public RectD(bool isValid)
    {
        if (isValid)
        {
            Left = 0; Top = 0; Right = 0; Bottom = 0;
        }
        else
        {
            Left = double.MaxValue; Top = double.MaxValue;
            Right = -double.MaxValue; Bottom = -double.MaxValue;
        }
    }
    public double Width
    {
        readonly get => Right - Left;
        set => Right = Left + value;
    }

    public double Height
    {
        readonly get => Bottom - Top;
        set => Bottom = Top + value;
    }

    public readonly bool IsEmpty()
    {
        return Clipper.LessThanOrEqual(Bottom, Top)  || Clipper.LessThanOrEqual(Right, Left);
    }

    public readonly PointD MidPoint()
    {
        return new PointD((Left + Right) / 2, (Top + Bottom) / 2);
    }

    public readonly bool Contains(PointD pt)
    {
        return Clipper.GreaterThan(pt.X, Left) && Clipper.LessThan(pt.X, Right) &&
              Clipper.GreaterThan(pt.Y, Top) && Clipper.LessThan(pt.Y, Bottom);
    }

    public readonly bool Contains(RectD rec)
    {
        return Clipper.GreaterThanOrEqual(rec.Left, Left) && Clipper.LessThanOrEqual(rec.Right, Right) &&
              Clipper.GreaterThanOrEqual(rec.Top, Top) && Clipper.LessThanOrEqual(rec.Bottom, Bottom);
    }

    public readonly bool Intersects(RectD rec)
    {
        return (Clipper.LessThan(Math.Max(Left, rec.Left), Math.Min(Right, rec.Right))) &&
               (Clipper.LessThan(Math.Max(Top, rec.Top), Math.Min(Bottom, rec.Bottom)));
    }

    public readonly PathD AsPath()
    {
        PathD result = new PathD(4)
        {
            new PointD(Left, Top),
            new PointD(Right, Top),
            new PointD(Right, Bottom),
            new PointD(Left, Bottom)
        };
        return result;
    }

}
