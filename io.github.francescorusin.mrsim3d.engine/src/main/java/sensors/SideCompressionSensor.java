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

import bodies.Voxel;
import engine.Ode4jEngine;

public class SideCompressionSensor implements Sensor {
  private final Voxel voxel;
  private final double[] range;

  public SideCompressionSensor(Voxel voxel, double range) {
    this.voxel = voxel;
    if (range < 0) {
      throw new IllegalArgumentException(
          String.format("Attempted to use invalid range (%.4f)", range));
    }
    this.range = new double[] {Math.max(1 - range, 0d), Math.min(1 + range, 2d)};
  }

  public SideCompressionSensor(Voxel voxel) {
    this(voxel, .5);
  }

  @Override
  public double[] sense(Ode4jEngine engine) {
    double average = Math.pow(voxel.currentVolume(engine.t()), 1d / 3d);
    double[] result = new double[12];
    int index = -1;
    for (Voxel.Edge edge : Voxel.Edge.values()) {
      result[++index] =
          voxel
                  .vertexBody(edge.v1)
                  .position(engine.t())
                  .vectorDistance(voxel.vertexBody(edge.v2).position(engine.t()))
                  .norm()
              / average;
      if (result[index] < range[0]) {
        result[index] = range[0];
      } else if (result[index] > range[1]) {
        result[index] = range[1];
      } else {
        result[index] = 2 * (result[index] - range[0]) / (range[1] - range[0]) - 1d;
      }
    }
    return result;
  }

  @Override
  public int outputSize() {
    return 12;
  }
}
