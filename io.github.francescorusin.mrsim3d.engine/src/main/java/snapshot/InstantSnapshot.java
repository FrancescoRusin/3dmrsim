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
package snapshot;

import engine.Ode4jEngine;
import viewer.Viewer;

import java.util.List;

public record InstantSnapshot(Ode4jEngine.Configuration configuration, List<BodySnapshot> activeBodies, List<BodySnapshot> passiveBodies, List<JointSnapshot> interbodyJoints, double t) implements AbstractSnapshot {
  @Override
  public void draw(Viewer viewer) {
    configuration.terrain().draw(viewer);
    for (BodySnapshot body : activeBodies) {
      body.draw(viewer);
    }
    for (BodySnapshot body : passiveBodies) {
      body.draw(viewer);
    }
    for (JointSnapshot joint : interbodyJoints) {
      joint.draw(viewer);
    }
  }
}
