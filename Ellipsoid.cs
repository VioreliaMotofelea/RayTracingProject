using System;


namespace rt
{
    public class Ellipsoid : Geometry
    {
        private Vector Center { get; }
        private Vector SemiAxesLength { get; }
        private double Radius { get; }
        
        public Quaternion Rotation { get; set;  } = Quaternion.NONE;
        
        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Material material, Color color) : base(material, color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Color color) : base(color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public Ellipsoid (Ellipsoid e) : this(new Vector(e.Center), new Vector(e.SemiAxesLength), e.Radius, new Material(e.Material), new Color(e.Color))
        {
        }

        public override Intersection GetIntersection(Line line, double minDist, double maxDist)
        {
            // TODO: ADD CODE HERE
            return Intersection.NONE;
        }
    }
}
