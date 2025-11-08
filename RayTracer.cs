using System;

namespace rt
{
    class RayTracer(Geometry[] geometries, Light[] lights)
    {
        private double ImageToViewPlane(int n, int imgSize, double viewPlaneSize)
        {
            return -n * viewPlaneSize / imgSize + viewPlaneSize / 2;
        }

        private Intersection FindFirstIntersection(Line ray, double minDist, double maxDist)
        {
            var intersection = Intersection.NONE;

            foreach (var geometry in geometries)
            {
                var intr = geometry.GetIntersection(ray, minDist, maxDist);

                if (!intr.Valid || !intr.Visible) continue;

                if (!intersection.Valid || !intersection.Visible)
                {
                    intersection = intr;
                }
                else if (intr.T < intersection.T)
                {
                    intersection = intr;
                }
            }

            return intersection;
        }

        private bool IsLit(Vector point, Light light)
        {
            const double epsilon = 1e-4;

            var toLight = new Vector(light.Position - point);
            var distance = toLight.Length();
            if (distance <= epsilon)
            {
                return true;
            }

            var direction = toLight / distance;
            var origin = new Vector(point + direction * epsilon);
            var ray = new Line(origin, origin + direction);
            var maxDistance = distance - epsilon;

            foreach (var geometry in geometries)
            {
                if (geometry is CtScan)
                {
                    continue;
                }

                var intersection = geometry.GetIntersection(ray, epsilon, maxDistance);
                if (!intersection.Valid || !intersection.Visible)
                {
                    continue;
                }

                if (intersection.T < maxDistance)
                {
                    return false;
                }
            }

            return true;
        }

        public void Render(Camera camera, int width, int height, string filename)
        {
            var background = new Color(0.2, 0.2, 0.2, 1.0);

            var image = new Image(width, height);
            CtScan.SceneLights = lights;
            camera.Normalize();

            var forward = new Vector(camera.Direction).Normalize();
            var up = new Vector(camera.Up).Normalize();
            var right = forward ^ up;
            right.Normalize();
            up = (right ^ forward).Normalize();

            var viewPlaneCenter = camera.Position + forward * camera.ViewPlaneDistance;
            var pixelWidth = camera.ViewPlaneWidth / width;
            var pixelHeight = camera.ViewPlaneHeight / height;

            const double epsilon = 1e-4;

            for (var i = 0; i < width; i++)
            {
                for (var j = 0; j < height; j++)
                {
                    var viewPlaneX = ImageToViewPlane(i, width, camera.ViewPlaneWidth) - pixelWidth * 0.5;
                    var viewPlaneY = ImageToViewPlane(j, height, camera.ViewPlaneHeight) - pixelHeight * 0.5;
                    var pixelPosition = viewPlaneCenter - right * viewPlaneX + up * viewPlaneY;

                    var ray = new Line(camera.Position, pixelPosition);

                    var firstOpaque = FindFirstIntersection(ray, camera.FrontPlaneDistance, camera.BackPlaneDistance);
                    var maxOpaqueDistance = camera.BackPlaneDistance;
                    if (firstOpaque.Valid && firstOpaque.Visible && firstOpaque.Geometry is not CtScan)
                    {
                        maxOpaqueDistance = Math.Min(firstOpaque.T - epsilon, maxOpaqueDistance);
                        if (maxOpaqueDistance < camera.FrontPlaneDistance)
                        {
                            maxOpaqueDistance = camera.FrontPlaneDistance;
                        }
                    }

                    var volumeColor = new Color();
                    var accumulatedAlpha = 0.0;

                    foreach (var geometry in geometries)
                    {
                        if (geometry is not CtScan volume)
                        {
                            continue;
                        }

                        var maxDistance = maxOpaqueDistance;
                        if (maxDistance <= camera.FrontPlaneDistance)
                        {
                            continue;
                        }

                        var volumeIntersection = volume.GetIntersection(ray, camera.FrontPlaneDistance, maxDistance);
                        if (!volumeIntersection.Valid)
                        {
                            continue;
                        }

                        var volColor = volumeIntersection.Color;
                        var alpha = Math.Max(0.0, Math.Min(1.0, volColor.Alpha));
                        if (alpha <= 0.0)
                        {
                            continue;
                        }

                        var transmittance = 1.0 - accumulatedAlpha;
                        if (transmittance <= 0.0)
                        {
                            break;
                        }

                        volumeColor += volColor * transmittance;
                        accumulatedAlpha += transmittance * alpha;

                        if (accumulatedAlpha > 0.995)
                        {
                            break;
                        }
                    }

                    var finalColor = background;

                    if (accumulatedAlpha < 0.995 && firstOpaque.Valid && firstOpaque.Visible && firstOpaque.Geometry is not CtScan)
                    {
                        var point = firstOpaque.Position;
                        var normal = new Vector(firstOpaque.Normal).Normalize();
                        var viewDirection = new Vector(camera.Position - point).Normalize();

                        var shadedColor = new Color();
                        foreach (var light in lights)
                        {
                            var ambient = firstOpaque.Material.Ambient * light.Ambient;
                            var lightContribution = ambient;

                            var toLight = new Vector(light.Position - point);
                            var lightDistance = toLight.Length();
                            if (lightDistance > epsilon)
                            {
                                toLight = toLight / lightDistance;
                                if (IsLit(point + normal * epsilon, light))
                                {
                                    var diffuseFactor = Math.Max(0.0, normal * toLight);
                                    if (diffuseFactor > 0.0)
                                    {
                                        lightContribution += (firstOpaque.Material.Diffuse * light.Diffuse) * diffuseFactor;
                                    }

                                    var halfVector = new Vector(toLight + viewDirection);
                                    halfVector.Normalize();
                                    var specFactor = Math.Pow(Math.Max(0.0, normal * halfVector), firstOpaque.Material.Shininess);
                                    if (specFactor > 0.0)
                                    {
                                        lightContribution += (firstOpaque.Material.Specular * light.Specular) * specFactor;
                                    }
                                }
                            }

                            shadedColor += lightContribution;
                        }

                        finalColor = shadedColor;
                    }

                    if (accumulatedAlpha > 0.0)
                    {
                        finalColor = volumeColor + finalColor * (1.0 - accumulatedAlpha);
                    }

                    var finalSystemColor = finalColor.ToSystemColor();
                    var finalClamped = new Color(finalSystemColor.Red / 255.0, finalSystemColor.Green / 255.0, finalSystemColor.Blue / 255.0, 1.0);
                    image.SetPixel(i, j, finalClamped);
                }
            }

            image.Store(filename);
        }
    }
}