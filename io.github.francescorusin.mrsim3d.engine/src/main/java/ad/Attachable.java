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
package ad;

import bodies.Body;
import bodies.SimulationObject;
import geometry.Vector3D;
import java.util.List;
import java.util.Map;
import java.util.Set;

public interface Attachable extends SimulationObject {
  // TODO MAYBE SWITCH LIST<BODY> WITH AN ARRAY OR A RECORD
  List<List<Body>> attachPossibilities();

  Map<List<Body>, Vector3D> attachPossibilitiesPositions(double t);

  Map<Body, Set<Body>> attachedBodies();

  boolean checkAttachment(Body body);

  default boolean checkAttachment(Attachable attachable) {
    for (Body body : attachable.bodyParts()) {
      if (checkAttachment(body)) {
        return true;
      }
    }
    return false;
  }
}
