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
import engine.Ode4jEngine;
import geometry.Vector3D;
import test.VisualTest;

import java.util.List;

public interface EmbodiedAgent {
  List<AbstractBody> components();

  List<Action> act(Ode4jEngine engine);

  void assemble(Ode4jEngine engine, Vector3D position);
  //TODO REPLACE DRAWER
  void draw(VisualTest test);
}
