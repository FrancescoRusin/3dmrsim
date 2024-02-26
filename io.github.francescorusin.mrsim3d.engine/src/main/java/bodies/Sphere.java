/*-
 * ========================LICENSE_START=================================
 * mrsim3d.engine
 * %%
 * Copyright (C) 2024 Francesco Rusin
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
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DMass;
import org.ode4j.ode.OdeHelper;

public class Sphere extends Body {
  private double radius;
  private DMass mass;
  private static double DEFAULT_MASS = 1d;

  public Sphere(double radius, double mass) {
    this.radius = radius;
    this.mass = OdeHelper.createMass();
    this.mass.setSphereTotal(mass, radius);
  }

  @Override
  public BoundingBox boundingBox() {
    DVector3C center = body.getPosition();
    return new BoundingBox(
        new Vector3D(center.get0() - radius, center.get1() - radius, center.get2() - radius),
        new Vector3D(center.get0() + radius, center.get1() + radius, center.get2() + radius));
  }

  public double getRadius() {
    return radius;
  }

  @Override
  public double mass() {
    return mass.getMass();
  }

  @Override
  public double volume() {
    return (double) 4 / 3 * Math.PI * Math.pow(radius, 3);
  }

  @Override
  public double[] angle() {
    return new double[]{0d, 0d};
  }

  @Override
  public void assemble(Ode4jEngine engine, Vector3D position) {
    body = OdeHelper.createBody(engine.getWorld());
    body.setPosition(position.x(), position.y(), position.z());
    body.setMass(mass);
    collisionGeometry = OdeHelper.createSphere(engine.getSpace(), radius);
    collisionGeometry.setBody(body);
  }
}
