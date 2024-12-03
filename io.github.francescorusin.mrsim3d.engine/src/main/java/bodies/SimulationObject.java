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

import java.util.List;

public interface SimulationObject {
    List<Body> bodyParts();

    BoundingBox boundingBox(double t);

    Vector3D position(double t);

    Vector3D velocity(double t);

    void rotate(Ode4jEngine engine, Vector3D eulerAngles);

    void translate(Ode4jEngine engine, Vector3D translation);

    void assemble(Ode4jEngine engine, Vector3D position);
}
