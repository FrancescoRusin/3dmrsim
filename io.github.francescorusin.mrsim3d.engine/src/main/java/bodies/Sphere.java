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
import org.ode4j.ode.OdeHelper;
import test.VisualTest;

import static drawstuff.DrawStuff.*;

public class Sphere extends Body {
  private final double radius;

  public Sphere(double radius, double mass) {
    this.radius = radius;
    this.mass = OdeHelper.createMass();
    this.mass.setSphereTotal(mass, radius);
  }

  @Override
  public BoundingBox boundingBox(double t) {
    DVector3C center = body.getPosition();
    return new BoundingBox(
        new Vector3D(center.get0() - radius, center.get1() - radius, center.get2() - radius),
        new Vector3D(center.get0() + radius, center.get1() + radius, center.get2() + radius));
  }

  public double radius() {
    return radius;
  }

  @Override
  public double currentVolume(double t) {
    return (double) 4 / 3 * Math.PI * Math.pow(radius, 3);
  }

  @Override
  public Vector3D angle(double t) {
    return new Vector3D();
  }

  @Override
  public void assemble(Ode4jEngine engine, Vector3D position) {
    body = OdeHelper.createBody(engine.world());
    collisionGeometry = OdeHelper.createSphere(engine.space(), radius);
    collisionGeometry.setBody(body);
    body.setPosition(position.x(), position.y(), position.z());
    body.setMass(mass);
  }

  @Override
  public void rotate(Vector3D eulerAngles) {}

  @Override
  public void draw(VisualTest test) {
    dsSetColor(0, 1, 0);
    dsSetTexture(DS_TEXTURE_NUMBER.DS_CHECKERED);
    dsDrawSphere(body.getPosition(), body.getRotation(), radius);
  }
}
