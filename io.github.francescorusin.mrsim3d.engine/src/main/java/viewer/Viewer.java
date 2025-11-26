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
import snapshot.InstantSnapshot;

import java.awt.*;
import java.util.function.Consumer;

public interface Viewer extends Consumer<InstantSnapshot> {

  int loadTexture(String filename);

  void drawTriangle(Vector3D v1, Vector3D v2, Vector3D v3, Color color);

  void drawTexture(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D v4, int texID, int hReps, int vReps);

  void drawSphere(Vector3D position, double radius, Color color);

  void drawLine(Vector3D p1, Vector3D p2, Color color);
}
