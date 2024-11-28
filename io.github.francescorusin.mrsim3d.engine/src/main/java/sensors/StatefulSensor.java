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

import engine.Ode4jEngine;
import java.util.Arrays;

public abstract class StatefulSensor implements Sensor {
  // sensors that do not sense by themselves and instead read some cached value updated by external
  // actions;
  // as different sensors potentially update differently, no standard state update function is
  // implemented
  protected double[] currentState;

  void resetReadings() {
    Arrays.fill(currentState, 0d);
  }

  @Override
  public double[] sense(Ode4jEngine engine) {
    double[] readingsCopy = new double[currentState.length];
    for (int i = 0; i < currentState.length; ++i) {
      readingsCopy[i] = Math.max(-1d, Math.min(1d, currentState[i]));
    }
    resetReadings();
    return readingsCopy;
  }

  @Override
  public int outputSize() {
    return currentState.length;
  }
}
