package bodies; /*-
                 * ========================LICENSE_START=================================
                 * mrsim3d.core
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

import geometry.BoundingBox;
import geometry.Vector3D;
import java.util.List;

public abstract class MultiBody extends Body {
  @Override
  public BoundingBox boundingBox() {
    return bodyParts().stream()
        .map(Body::boundingBox)
        .reduce(BoundingBox::enclosing)
        .orElseThrow();
  }

  abstract List<Body> bodyParts();

  @Override
  public double mass() {
    return bodyParts().stream().mapToDouble(Body::mass).sum();
  }

  @Override
  public Vector3D position() {
    return bodyParts().stream()
        .map(b -> b.position().times(b.mass()))
        .reduce(Vector3D::sum)
        .orElseThrow();
  }

  @Override
  public Vector3D velocity() {
    return bodyParts().stream()
        .map(b -> b.velocity().times(b.mass()))
        .reduce(Vector3D::sum)
        .orElseThrow();
  }
}
