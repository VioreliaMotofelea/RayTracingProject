using System;
using System.IO;
using System.Text.RegularExpressions;

namespace rt;

public class CtScan: Geometry
{
    internal static Light[] SceneLights { get; set; } = Array.Empty<Light>();

    private readonly Vector _position;
    private readonly double _scale;
    private readonly ColorMap _colorMap;
    private readonly byte[] _data;

    private readonly int[] _resolution = new int[3];
    private readonly double[] _thickness = new double[3];
    private readonly Vector _v0;
    private readonly Vector _v1;

    public CtScan(string datFile, string rawFile, Vector position, double scale, ColorMap colorMap) : base(Color.NONE)
    {
        _position = position;
        _scale = scale;
        _colorMap = colorMap;

        var lines = File.ReadLines(datFile);
        foreach (var line in lines)
        {
            var kv = Regex.Replace(line, "[:\\t ]+", ":").Split(":");
            if (kv[0] == "Resolution")
            {
                _resolution[0] = Convert.ToInt32(kv[1]);
                _resolution[1] = Convert.ToInt32(kv[2]);
                _resolution[2] = Convert.ToInt32(kv[3]);
            } else if (kv[0] == "SliceThickness")
            {
                _thickness[0] = Convert.ToDouble(kv[1]);
                _thickness[1] = Convert.ToDouble(kv[2]);
                _thickness[2] = Convert.ToDouble(kv[3]);
            }
        }

        _v0 = position;
        _v1 = position + new Vector(_resolution[0]*_thickness[0]*scale, _resolution[1]*_thickness[1]*scale, _resolution[2]*_thickness[2]*scale);

        var len = _resolution[0] * _resolution[1] * _resolution[2];
        _data = new byte[len];
        using FileStream f = new FileStream(rawFile, FileMode.Open, FileAccess.Read);
        if (f.Read(_data, 0, len) != len)
        {
            throw new InvalidDataException($"Failed to read the {len}-byte raw data");
        }
    }
    
    private ushort Value(int x, int y, int z)
    {
        if (x < 0 || y < 0 || z < 0 || x >= _resolution[0] || y >= _resolution[1] || z >= _resolution[2])
        {
            return 0;
        }

        return _data[z * _resolution[1] * _resolution[0] + y * _resolution[0] + x];
    }

    public override Intersection GetIntersection(Line line, double minDist, double maxDist)
    {
        const double epsilon = 1e-8;

        var boundsMin = new Vector(
            Math.Min(_v0.X, _v1.X),
            Math.Min(_v0.Y, _v1.Y),
            Math.Min(_v0.Z, _v1.Z));
        var boundsMax = new Vector(
            Math.Max(_v0.X, _v1.X),
            Math.Max(_v0.Y, _v1.Y),
            Math.Max(_v0.Z, _v1.Z));

        var origin = line.X0;
        var direction = line.Dx;

        var tMin = minDist;
        var tMax = maxDist;

        double[] originComponents = { origin.X, origin.Y, origin.Z };
        double[] directionComponents = { direction.X, direction.Y, direction.Z };
        double[] minComponents = { boundsMin.X, boundsMin.Y, boundsMin.Z };
        double[] maxComponents = { boundsMax.X, boundsMax.Y, boundsMax.Z };

        for (var axis = 0; axis < 3; axis++)
        {
            var dir = directionComponents[axis];
            var orig = originComponents[axis];
            var minValue = minComponents[axis];
            var maxValue = maxComponents[axis];

            if (Math.Abs(dir) < epsilon)
            {
                if (orig < minValue || orig > maxValue)
                {
                    return Intersection.NONE;
                }
                continue;
            }

            var invDir = 1.0 / dir;
            var t0 = (minValue - orig) * invDir;
            var t1 = (maxValue - orig) * invDir;
            if (t0 > t1)
            {
                (t0, t1) = (t1, t0);
            }

            tMin = Math.Max(tMin, t0);
            tMax = Math.Min(tMax, t1);

            if (tMin > tMax)
            {
                return Intersection.NONE;
            }
        }

        var entryT = Math.Max(tMin, minDist);
        var exitT = Math.Min(tMax, maxDist);

        if (exitT <= entryT)
        {
            return Intersection.NONE;
        }

        var voxelStep = Math.Min(_thickness[0], Math.Min(_thickness[1], _thickness[2])) * _scale * 0.5;
        if (voxelStep <= epsilon)
        {
            return Intersection.NONE;
        }

        double accumR = 0.0;
        double accumG = 0.0;
        double accumB = 0.0;
        double accumAlpha = 0.0;

        var dx = _thickness[0] * _scale;
        var dy = _thickness[1] * _scale;
        var dz = _thickness[2] * _scale;

        double SampleDensity(Vector position)
        {
            var lx = (position.X - _position.X) / dx;
            var ly = (position.Y - _position.Y) / dy;
            var lz = (position.Z - _position.Z) / dz;

            var ix = (int)Math.Floor(lx);
            var iy = (int)Math.Floor(ly);
            var iz = (int)Math.Floor(lz);

            var fx = lx - ix;
            var fy = ly - iy;
            var fz = lz - iz;

            double c000 = Value(ix, iy, iz);
            double c100 = Value(ix + 1, iy, iz);
            double c010 = Value(ix, iy + 1, iz);
            double c110 = Value(ix + 1, iy + 1, iz);
            double c001 = Value(ix, iy, iz + 1);
            double c101 = Value(ix + 1, iy, iz + 1);
            double c011 = Value(ix, iy + 1, iz + 1);
            double c111 = Value(ix + 1, iy + 1, iz + 1);

            var c00 = c000 * (1 - fx) + c100 * fx;
            var c10 = c010 * (1 - fx) + c110 * fx;
            var c01 = c001 * (1 - fx) + c101 * fx;
            var c11 = c011 * (1 - fx) + c111 * fx;

            var c0 = c00 * (1 - fy) + c10 * fy;
            var c1 = c01 * (1 - fy) + c11 * fy;

            return c0 * (1 - fz) + c1 * fz;
        }

        Vector SampleGradient(Vector position)
        {
            var px1 = Math.Min(position.X + dx, boundsMax.X);
            var px0 = Math.Max(position.X - dx, boundsMin.X);
            var py1 = Math.Min(position.Y + dy, boundsMax.Y);
            var py0 = Math.Max(position.Y - dy, boundsMin.Y);
            var pz1 = Math.Min(position.Z + dz, boundsMax.Z);
            var pz0 = Math.Max(position.Z - dz, boundsMin.Z);

            var gx = SampleDensity(new Vector(px1, position.Y, position.Z)) - SampleDensity(new Vector(px0, position.Y, position.Z));
            var gy = SampleDensity(new Vector(position.X, py1, position.Z)) - SampleDensity(new Vector(position.X, py0, position.Z));
            var gz = SampleDensity(new Vector(position.X, position.Y, pz1)) - SampleDensity(new Vector(position.X, position.Y, pz0));

            return new Vector(gx / (Math.Max(px1 - px0, epsilon)), gy / (Math.Max(py1 - py0, epsilon)), gz / (Math.Max(pz1 - pz0, epsilon)));
        }

        for (var t = entryT; t <= exitT && accumAlpha < 0.995; t += voxelStep)
        {
            var samplePosition = line.CoordinateToPosition(t);
            if (samplePosition.X < boundsMin.X - epsilon || samplePosition.X > boundsMax.X + epsilon ||
                samplePosition.Y < boundsMin.Y - epsilon || samplePosition.Y > boundsMax.Y + epsilon ||
                samplePosition.Z < boundsMin.Z - epsilon || samplePosition.Z > boundsMax.Z + epsilon)
            {
                continue;
            }

            var density = SampleDensity(samplePosition);
            if (density <= 0.0)
            {
                continue;
            }

            var mappedColor = _colorMap.GetColor((ushort)Math.Clamp((int)Math.Round(density), 0, ushort.MaxValue));
            var alpha = Math.Max(0.0, Math.Min(1.0, mappedColor.Alpha));
            if (alpha <= 0.0)
            {
                continue;
            }

            var transmittance = 1.0 - accumAlpha;
            if (transmittance <= 0.0)
            {
                break;
            }

            var baseColor = mappedColor.ToSystemColor();
            var baseR = baseColor.Red / 255.0;
            var baseG = baseColor.Green / 255.0;
            var baseB = baseColor.Blue / 255.0;

            double litR;
            double litG;
            double litB;

            var gradient = SampleGradient(samplePosition);
            var hasNormal = gradient.Length() > 1e-6;
            if (hasNormal)
            {
                gradient.Normalize();
            }

            if (SceneLights.Length == 0 || !hasNormal)
            {
                const double ambientOnly = 0.15;
                litR = baseR * ambientOnly;
                litG = baseG * ambientOnly;
                litB = baseB * ambientOnly;
            }
            else
            {
                double ambientFactor = 0.0;
                double diffR = 0.0;
                double diffG = 0.0;
                double diffB = 0.0;

                foreach (var light in SceneLights)
                {
                    var ambientColor = light.Ambient.ToSystemColor();
                    ambientFactor += (ambientColor.Red + ambientColor.Green + ambientColor.Blue) / (3.0 * 255.0);

                    var toLight = new Vector(light.Position - samplePosition);
                    if (toLight.Length() < epsilon) continue;
                    toLight.Normalize();

                    var ndotl = Math.Max(0.0, gradient * toLight);
                    if (ndotl <= 0.0) continue;

                    var diffuseColor = light.Diffuse.ToSystemColor();
                    diffR += (diffuseColor.Red / 255.0) * ndotl;
                    diffG += (diffuseColor.Green / 255.0) * ndotl;
                    diffB += (diffuseColor.Blue / 255.0) * ndotl;
                }

                var lightCount = SceneLights.Length;
                ambientFactor = ambientFactor / Math.Max(lightCount, 1) * 0.2 + 0.05;
                diffR = diffR / Math.Max(lightCount, 1);
                diffG = diffG / Math.Max(lightCount, 1);
                diffB = diffB / Math.Max(lightCount, 1);

                litR = baseR * Math.Clamp(ambientFactor + diffR, 0.0, 1.0);
                litG = baseG * Math.Clamp(ambientFactor + diffG, 0.0, 1.0);
                litB = baseB * Math.Clamp(ambientFactor + diffB, 0.0, 1.0);
            }

            litR = Math.Clamp(litR, 0.0, 1.0);
            litG = Math.Clamp(litG, 0.0, 1.0);
            litB = Math.Clamp(litB, 0.0, 1.0);

            var weight = transmittance * alpha;
            accumR += litR * weight;
            accumG += litG * weight;
            accumB += litB * weight;
            accumAlpha += weight;
        }

        if (accumAlpha <= 0.0)
        {
            return Intersection.NONE;
        }

        accumR = Math.Clamp(accumR, 0.0, 1.0);
        accumG = Math.Clamp(accumG, 0.0, 1.0);
        accumB = Math.Clamp(accumB, 0.0, 1.0);
        accumAlpha = Math.Clamp(accumAlpha, 0.0, 1.0);

        var finalColor = new Color(accumR, accumG, accumB, accumAlpha);
        return new Intersection(true, false, this, line, entryT, new Vector(), Material.BLANK, finalColor);
    }
    
    private int[] GetIndexes(Vector v)
    {
        return new []{
            (int)Math.Floor((v.X - _position.X) / _thickness[0] / _scale), 
            (int)Math.Floor((v.Y - _position.Y) / _thickness[1] / _scale),
            (int)Math.Floor((v.Z - _position.Z) / _thickness[2] / _scale)};
    }
    private Color GetColor(Vector v)
    {
        int[] idx = GetIndexes(v);

        ushort value = Value(idx[0], idx[1], idx[2]);
        return _colorMap.GetColor(value);
    }

    private Vector GetNormal(Vector v)
    {
        int[] idx = GetIndexes(v);
        double x0 = Value(idx[0] - 1, idx[1], idx[2]);
        double x1 = Value(idx[0] + 1, idx[1], idx[2]);
        double y0 = Value(idx[0], idx[1] - 1, idx[2]);
        double y1 = Value(idx[0], idx[1] + 1, idx[2]);
        double z0 = Value(idx[0], idx[1], idx[2] - 1);
        double z1 = Value(idx[0], idx[1], idx[2] + 1);

        return new Vector(x1 - x0, y1 - y0, z1 - z0).Normalize();
    }
}