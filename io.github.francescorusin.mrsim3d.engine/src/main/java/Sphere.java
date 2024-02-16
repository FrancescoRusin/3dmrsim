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
import geometry.BoundingBox;
import geometry.Vector3D;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.OdeHelper;

public final class Sphere extends RigidBody {
  DSphere sphere;
  Vector3D center;
  double radius;
  double mass;

  public Sphere(Vector3D center, double radius, double mass) {
    this.center = center;
    this.radius = radius;
    this.mass = mass;
    this.sphere = OdeHelper.createSphere(radius);
  }

  @Override
  public BoundingBox boundingBox() {

    return new BoundingBox(
        new Vector3D(center.x() - radius, center.y() - radius, center.z() - radius),
        new Vector3D(center.x() + radius, center.y() + radius, center.z() + radius));
  }

  @Override
  public double mass() {
    return mass;
  }

  @Override
  public void assemble(Engine engine, Vector3D position) {
    // TODO
  }
}
