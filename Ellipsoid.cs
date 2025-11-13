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
            var rotation = Rotation ?? Quaternion.NONE;
            var localRotation = new Quaternion(rotation.W, rotation.X, rotation.Y, rotation.Z).Normalize();
            var inverseRotation = new Quaternion(localRotation.W, -localRotation.X, -localRotation.Y, -localRotation.Z);

            var axes = SemiAxesLength * Radius;
            var epsilon = 1e-8;
            if (Math.Abs(axes.X) <= epsilon || Math.Abs(axes.Y) <= epsilon || Math.Abs(axes.Z) <= epsilon)
            {
                return Intersection.NONE;
            }

            var originToCenter = new Vector(line.X0 - Center);
            originToCenter.Rotate(inverseRotation);
            var directionLocal = new Vector(line.Dx);
            directionLocal.Rotate(inverseRotation);

            var originScaled = new Vector(originToCenter);
            originScaled.Divide(axes);
            var directionScaled = new Vector(directionLocal);
            directionScaled.Divide(axes);

            var a = directionScaled * directionScaled;
            if (Math.Abs(a) <= epsilon)
            {
                return Intersection.NONE;
            }

            var b = 2.0 * (originScaled * directionScaled);
            var c = originScaled * originScaled - 1.0;
            var discriminant = b * b - 4.0 * a * c;
            if (discriminant < 0.0)
            {
                return Intersection.NONE;
            }

            var sqrtDiscriminant = Math.Sqrt(discriminant);
            var denom = 0.5 / a;
            var t0 = (-b - sqrtDiscriminant) * denom;
            var t1 = (-b + sqrtDiscriminant) * denom;

            double hitT = double.PositiveInfinity;
            var minThreshold = Math.Max(minDist, epsilon);

            if (t0 >= minThreshold && t0 <= maxDist)
            {
                hitT = t0;
            }
            else if (t1 >= minThreshold && t1 <= maxDist)
            {
                hitT = t1;
            }

            if (!double.IsFinite(hitT) || hitT == double.PositiveInfinity)
            {
                return Intersection.NONE;
            }

            var localHitUnscaled = new Vector(originToCenter + directionLocal * hitT);
            var normalLocal = new Vector(
                localHitUnscaled.X / (axes.X * axes.X),
                localHitUnscaled.Y / (axes.Y * axes.Y),
                localHitUnscaled.Z / (axes.Z * axes.Z));
            normalLocal.Rotate(localRotation);
            normalLocal.Normalize();

            return new Intersection(true, true, this, line, hitT, normalLocal, Material, Color);
        }
    }
}
