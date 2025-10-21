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
package viewer;

import geometry.Vector3D;

import java.awt.*;

public abstract class Viewer {
  public enum Mode {
    DISPLAY, DEBUG
  }

  public final Mode mode;

  Viewer(Mode mode) {
    this.mode = mode;
  }

  public abstract int loadTexture(String filename);

  public abstract void drawTriangle(Vector3D v1, Vector3D v2, Vector3D v3, Color color);

  public abstract void drawTexture(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D v4, int texID, int hReps, int vReps);

  public abstract void drawSphere(Vector3D position, double radius, Color color);

  public abstract void drawLine(Vector3D p1, Vector3D p2, Color color);
}
