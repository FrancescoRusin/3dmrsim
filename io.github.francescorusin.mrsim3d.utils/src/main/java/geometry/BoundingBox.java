package geometry; /*-
                   * ========================LICENSE_START=================================
                   * core
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

import java.util.Arrays;

public record BoundingBox(Vector3D min, Vector3D max) {
  public static BoundingBox enclosing(BoundingBox... boxes) {
    return Arrays.stream(boxes)
        .sequential()
        .reduce(
            (b1, b2) ->
                new BoundingBox(
                    new Vector3D(
                        Math.min(b1.min.x(), b2.min.x()),
                        Math.min(b1.min.y(), b2.min.y()),
                        Math.min(b1.min.z(), b2.min.z())),
                    new Vector3D(
                        Math.max(b1.max.x(), b2.max.x()),
                        Math.max(b1.max.y(), b2.max.y()),
                        Math.max(b1.max.z(), b2.max.z()))))
        .orElseThrow(
            () -> new IllegalArgumentException("Called enclosing on an empty bounding box set"));
  }
}
