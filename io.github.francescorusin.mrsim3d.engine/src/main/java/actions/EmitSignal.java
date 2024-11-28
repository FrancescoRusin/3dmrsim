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
package actions;

import bodies.SignalEmitter;
import engine.Ode4jEngine;
import geometry.Vector3D;
import viewer.Viewer;

public class EmitSignal implements Action {
  final SignalEmitter emitter;
  final Vector3D direction;
  final int channel;
  final double value;

  public EmitSignal(SignalEmitter emitter, Vector3D direction, int channel, double value) {
    this.emitter = emitter;
    this.direction = direction;
    this.channel = channel;
    this.value = value;
  }

  @Override
  public void execute(Ode4jEngine engine) {
    engine.emitSignal(emitter, direction, channel, value);
  }

  @Override
  public void draw(Viewer viewer) {
    // TODO IMPLEMENT
  }
}
