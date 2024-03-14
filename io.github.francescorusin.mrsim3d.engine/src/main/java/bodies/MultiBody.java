package bodies; /*-
                 * ========================LICENSE_START=================================
                 * mrsim3d.core
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
import java.util.List;

import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;

public abstract class MultiBody implements AbstractBody {
  @Override
  public BoundingBox boundingBox(double t) {
    return bodyParts().stream().map(b -> b.boundingBox(t)).reduce(BoundingBox::enclosing).orElseThrow();
  }

  public abstract List<Body> bodyParts();

  public abstract List<? extends DJoint> internalJoints();

  @Override
  public double mass() {
    return bodyParts().stream().mapToDouble(Body::mass).sum();
  }

  @Override
  public Vector3D position(double t) {
    return bodyParts().stream()
        .map(b -> b.position(t).times(b.mass()))
        .reduce(Vector3D::sum)
        .orElseThrow().times(1d / mass());
  }

  @Override
  public void translate(Ode4jEngine engine, Vector3D translation) {
    for (Body body : bodyParts()) {
      body.translate(engine, translation);
    }
  }

  @Override
  public Vector3D velocity(double t) {
    return bodyParts().stream()
        .map(b -> b.velocity(t).times(b.mass()))
        .reduce(Vector3D::sum)
        .orElseThrow();
  }
}
