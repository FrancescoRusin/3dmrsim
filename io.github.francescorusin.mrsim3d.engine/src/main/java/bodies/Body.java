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
import geometry.BoundingBox;
import geometry.Vector3D;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;

public abstract class Body {
  protected DBody body;
  protected DGeom collisionGeometry;
  public abstract BoundingBox boundingBox();

  public abstract double mass();

  public abstract double volume();

  public Vector3D position() {
    return new Vector3D(
        body.getPosition().get0(), body.getPosition().get1(), body.getPosition().get2());
  }

  public Vector3D velocity() {
    return new Vector3D(
        body.getLinearVel().get0(), body.getLinearVel().get1(), body.getLinearVel().get2());
  }

  public DBody getBody() {
    return body;
  }

  public void setBody(DBody body) {
    this.body = body;
  }

  public abstract void assemble(Ode4jEngine engine, Vector3D position);
}
