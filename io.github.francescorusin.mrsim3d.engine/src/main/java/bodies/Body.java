package bodies; /*-
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

import engine.Ode4jEngine;
import geometry.Vector3D;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DMass;

public abstract class Body implements AbstractBody {
  protected DBody body;
  protected DMass mass;
  protected DGeom collisionGeometry;

  @Override
  public double mass() {
    return mass.getMass();
  }

  public DGeom collisionGeometry() {
    return collisionGeometry;
  }

  @Override
  public Vector3D position(double t) {
    return new Vector3D(body.getPosition().get0(), body.getPosition().get1(), body.getPosition().get2());
  }

  @Override
  public void translate(Ode4jEngine engine, Vector3D translation) {
    body.setPosition(
            body.getPosition().get0() + translation.x(),
            body.getPosition().get1() + translation.y(),
            body.getPosition().get2() + translation.z()
    );
  }

  @Override
  public Vector3D velocity(double t) {
    return new Vector3D(body.getLinearVel().get0(), body.getLinearVel().get1(), body.getLinearVel().get2());
  }

  public DBody dBody() {
    return body;
  }
}