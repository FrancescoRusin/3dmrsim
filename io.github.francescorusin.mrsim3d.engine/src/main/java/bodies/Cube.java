/*-
 * ========================LICENSE_START=================================
 * mrsim3d.engine
 * %%
 * Copyright (C) 2024 - 2025 Francesco Rusin
 * %%
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * =========================LICENSE_END==================================
 */
package bodies;

import engine.Ode4jEngine;
import geometry.BoundingBox;
import geometry.Vector3D;

import java.awt.*;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.stream.Stream;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.ode.OdeHelper;
import snapshot.BodySnapshot;
import viewer.Viewer;

public class Cube extends Body {
    private final double sideLength;

    private enum Cache {
        ANGLE,
        BBOX
    }

    private final EnumMap<Cache, Double> cacheTime;
    private Vector3D angleCacher;
    private BoundingBox boundingBoxCacher;

    public Cube(double sideLength, double mass) {
        this.sideLength = sideLength;
        this.mass = OdeHelper.createMass();
        this.mass.setBoxTotal(mass, sideLength, sideLength, sideLength);
        cacheTime = new EnumMap<>(Cache.class);
    }

    public double sideLength() {
        return sideLength;
    }

    @Override
    public double currentVolume(double t) {
        return sideLength * sideLength * sideLength;
    }

    @Override
    public BoundingBox boundingBox(double t) {
        if (cacheTime.get(Cache.BBOX) != t) {
            cacheTime.put(Cache.BBOX, t);
            Vector3D angle = angle(t);
            List<Vector3D> rotatedVertices =
                    Stream.of(
                                    new Vector3D(1d, 1d, 1d),
                                    new Vector3D(-1d, 1d, 1d),
                                    new Vector3D(1d, -1d, 1d),
                                    new Vector3D(1d, 1d, -1d))
                            .map(v -> v.times(.5 * sideLength).rotate(angle))
                            .toList();
            double[] mins = new double[3];
            double[] maxs = new double[3];
            Arrays.fill(mins, Double.POSITIVE_INFINITY);
            Arrays.fill(maxs, Double.NEGATIVE_INFINITY);
            for (Vector3D(double x, double y, double z) : rotatedVertices) {
                if (Math.abs(x) > maxs[0]) {
                    maxs[0] = Math.abs(x);
                }
                if (Math.abs(y) > maxs[1]) {
                    maxs[1] = Math.abs(y);
                }
                if (Math.abs(z) > maxs[2]) {
                    maxs[2] = Math.abs(z);
                }
                if (-Math.abs(x) < mins[0]) {
                    mins[0] = -Math.abs(x);
                }
                if (-Math.abs(y) < mins[1]) {
                    mins[1] = -Math.abs(y);
                }
                if (-Math.abs(z) < mins[2]) {
                    mins[2] = -Math.abs(z);
                }
            }
            Vector3D center = position(t);
            mins[0] += center.x();
            maxs[0] += center.x();
            mins[1] += center.y();
            maxs[1] += center.y();
            mins[2] += center.z();
            maxs[2] += center.z();
            boundingBoxCacher = new BoundingBox(new Vector3D(mins), new Vector3D(maxs));
        }
        return boundingBoxCacher;
    }

    @Override
    public Vector3D angle(double t) {
        if (cacheTime.get(Cache.ANGLE) != t) {
            cacheTime.put(Cache.ANGLE, t);
            DMatrix3C rotationMatrix = body.getRotation();
            angleCacher =
                    new Vector3D(
                            Math.atan2(rotationMatrix.get21(), rotationMatrix.get22()),
                            Math.asin(-rotationMatrix.get20()),
                            Math.atan2(rotationMatrix.get10(), rotationMatrix.get00()));
        }
        return angleCacher;
    }

    @Override
    public void assemble(Ode4jEngine engine, Vector3D position) {
        for (Cache c : Cache.values()) {
            cacheTime.put(c, -1d);
        }
        body = OdeHelper.createBody(engine.world());
        body.setPosition(position.x(), position.y(), position.z());
        body.setMass(mass);
        collisionGeometry = OdeHelper.createBox(engine.bodySpace(), sideLength, sideLength, sideLength);
        collisionGeometry.setBody(body);
    }

    @Override
    public void rotate(Ode4jEngine engine, Vector3D eulerAngles) {
        Vector3D[] rotationBase =
                new Vector3D[] {
                        new Vector3D(1d, 0d, 0d).rotate(eulerAngles),
                        new Vector3D(0d, 1d, 0d).rotate(eulerAngles),
                        new Vector3D(0d, 0d, 1d).rotate(eulerAngles)
                };
        DMatrix3 rotationMatrix =
                new DMatrix3(
                        rotationBase[0].x(),
                        rotationBase[1].x(),
                        rotationBase[2].x(),
                        rotationBase[0].y(),
                        rotationBase[1].y(),
                        rotationBase[2].y(),
                        rotationBase[0].z(),
                        rotationBase[1].z(),
                        rotationBase[2].z());
        body.setRotation(new DMatrix3().eqMul(rotationMatrix, body.getRotation()));
    }

    public record CubeSnapshot(
            double sideLength, double mass, Vector3D position, Vector3D rotation, Vector3D velocity, Ode4jEngine.Mode mode
    )
            implements BodySnapshot {
        private final static Color CUBE_COLOR = Color.RED;
        @Override
        public void draw(Viewer viewer) {
            Vector3D[] vs = new Vector3D[8];
            int index = -1;
            for (Vector3D v : List.of(
                    new Vector3D(-1d, -1d, -1d),
                    new Vector3D(-1d, 1d, -1d),
                    new Vector3D(1d, 1d, -1d),
                    new Vector3D(1d, -1d, -1d),
                    new Vector3D(-1d, -1d, 1d),
                    new Vector3D(-1d, 1d, 1d),
                    new Vector3D(1d, 1d, 1d),
                    new Vector3D(1d, -1d, 1d)
            )) {
                vs[++index] = v.times(.5 * sideLength).rotate(rotation).sum(position);
            }
            viewer.drawTriangle(vs[0], vs[1], vs[2], CUBE_COLOR);
            viewer.drawTriangle(vs[0], vs[2], vs[3], CUBE_COLOR);
            viewer.drawTriangle(vs[0], vs[3], vs[7], CUBE_COLOR);
            viewer.drawTriangle(vs[0], vs[4], vs[7], CUBE_COLOR);
            viewer.drawTriangle(vs[0], vs[1], vs[5], CUBE_COLOR);
            viewer.drawTriangle(vs[0], vs[4], vs[5], CUBE_COLOR);
            viewer.drawTriangle(vs[1], vs[2], vs[5], CUBE_COLOR);
            viewer.drawTriangle(vs[2], vs[5], vs[6], CUBE_COLOR);
            viewer.drawTriangle(vs[2], vs[3], vs[7], CUBE_COLOR);
            viewer.drawTriangle(vs[2], vs[6], vs[7], CUBE_COLOR);
            viewer.drawTriangle(vs[4], vs[5], vs[6], CUBE_COLOR);
            viewer.drawTriangle(vs[4], vs[6], vs[7], CUBE_COLOR);
            if (mode == Ode4jEngine.Mode.DEBUG) {
                viewer.drawLine(vs[0], vs[1], Color.BLACK);
                viewer.drawLine(vs[1], vs[2], Color.BLACK);
                viewer.drawLine(vs[2], vs[3], Color.BLACK);
                viewer.drawLine(vs[0], vs[3], Color.BLACK);
                viewer.drawLine(vs[0], vs[4], Color.BLACK);
                viewer.drawLine(vs[1], vs[5], Color.BLACK);
                viewer.drawLine(vs[2], vs[6], Color.BLACK);
                viewer.drawLine(vs[3], vs[7], Color.BLACK);
                viewer.drawLine(vs[4], vs[5], Color.BLACK);
                viewer.drawLine(vs[5], vs[6], Color.BLACK);
                viewer.drawLine(vs[6], vs[7], Color.BLACK);
                viewer.drawLine(vs[4], vs[7], Color.BLACK);
            }
        }
    }

    @Override
    public BodySnapshot snapshot(Ode4jEngine engine, Ode4jEngine.Mode mode) {
        return new CubeSnapshot(
                this.sideLength,
                this.mass(),
                this.position(engine.t()),
                this.angle(engine.t()),
                this.velocity(engine.t()),
                mode
        );
    }
}
