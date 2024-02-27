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

public record Vector3D(double x, double y, double z) {
  public Vector3D() {
    this(0d, 0d, 0d);
  }

  public double norm() {
    return Math.sqrt(x * x + y * y + z * z);
  }

  public Vector3D times(double d) {
    return new Vector3D(this.x * d, this.y * d, this.z * d);
  }

  public Vector3D sum(Vector3D otherVector) {
    return new Vector3D(this.x + otherVector.x, this.y + otherVector.y, this.z + otherVector.z);
  }

  public Vector3D vectorDistance(Vector3D otherVector) {
    return new Vector3D(otherVector.x - this.x, otherVector.y - this.y, otherVector.z - this.z);
  }

  public double scalarProduct(Vector3D otherVector) {
    return x * otherVector.x + y * otherVector.y + z * otherVector.z;
  }

  public Vector3D vectorProduct(Vector3D otherVector) {
    return new Vector3D(
        this.y * otherVector.z - this.z * otherVector.y,
        this.z * otherVector.x - this.x * otherVector.z,
        this.x * otherVector.y - this.y * otherVector.x);
  }

  public String toString() {
    return String.format("[%f, %f, %f]", x, y, z);
  }
}
