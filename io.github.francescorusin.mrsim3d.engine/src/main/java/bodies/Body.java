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

import geometry.Vector3D;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DMass;

public abstract class Body implements AbstractBody {
  protected DBody body;
  protected DMass mass;
  protected DGeom collisionGeometry;

  @Override
  public Vector3D position() {
    return new Vector3D(
        body.getPosition().get0(), body.getPosition().get1(), body.getPosition().get2());
  }

  @Override
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

  public DGeom getCollisionGeometry() {
    return collisionGeometry;
  }

  public void setCollisionGeometry(DGeom collisionGeometry) {
    this.collisionGeometry = collisionGeometry;
  }
}
