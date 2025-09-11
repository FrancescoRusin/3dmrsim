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
package agents;

import actions.Action;
import bodies.AbstractBody;
import bodies.Body;
import bodies.SimulationObject;
import engine.Ode4jEngine;
import geometry.BoundingBox;
import geometry.Vector3D;
import java.util.Collection;
import java.util.List;

public interface EmbodiedAgent extends SimulationObject {
  List<AbstractBody> components();

  @Override
  default List<Body> bodyParts() {
    return components().stream().map(AbstractBody::bodyParts).flatMap(Collection::stream).toList();
  }

  @Override
  default BoundingBox boundingBox(double t) {
    List<AbstractBody> components = components();
    return components.stream()
        .map(b -> b.boundingBox(t))
        .reduce(BoundingBox::enclosing)
        .orElseThrow();
  }

  @Override
  default Vector3D position(double t) {
    List<AbstractBody> components = components();
    return components.stream()
        .map(b -> b.position(t))
        .reduce(Vector3D::sum)
        .orElseThrow()
        .times(1d / components.size());
  }

  @Override
  default Vector3D velocity(double t) {
    List<AbstractBody> components = components();
    return components.stream()
        .map(b -> b.velocity(t))
        .reduce(Vector3D::sum)
        .orElseThrow()
        .times(1d / components.size());
  }

  @Override
  default void rotate(Ode4jEngine engine, Vector3D eulerAngles) {
    Vector3D center = position(engine.t());
    for (AbstractBody component : components()) {
      Vector3D relativePosition = component.position(engine.t()).vectorDistance(center);
      component.translate(
          engine, relativePosition.rotate(eulerAngles).vectorDistance(relativePosition));
      component.rotate(engine, eulerAngles);
    }
  }

  @Override
  default void translate(Ode4jEngine engine, Vector3D translation) {
    for (AbstractBody component : components()) {
      component.translate(engine, translation);
    }
  }

  List<Action> act(Ode4jEngine engine);
}
