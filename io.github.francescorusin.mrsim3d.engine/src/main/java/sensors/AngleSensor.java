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
package sensors;

import bodies.AbstractBody;
import engine.Ode4jEngine;
import geometry.Vector3D;

public final class AngleSensor implements Sensor {
  private final AbstractBody body;

  public AngleSensor(AbstractBody body) {
    this.body = body;
  }

  @Override
  public double[] sense(Ode4jEngine engine) {
    Vector3D vectorAngle = body.angle(engine.t());
    double[] arrayAngle = new double[3];
    arrayAngle[0] = vectorAngle.x() / Math.PI;
    arrayAngle[1] = vectorAngle.y() / Math.PI;
    arrayAngle[2] = vectorAngle.z() / Math.PI;
    return arrayAngle;
  }

  @Override
  public int outputSize() {
    return 3;
  }
}
