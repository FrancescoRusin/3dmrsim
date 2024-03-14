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

  public Vector3D(double[] array) {
    this(array[0], array[1], array[2]);
    if (array.length != 3) {
      throw new IllegalArgumentException(String.format("Attempted to construct a Vector3D with the wrong number of parameters (%d)", array.length));
    }
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

  public Vector3D vectorDistance(Vector3D origin) {
    return new Vector3D(this.x - origin.x, this.y - origin.y, this.z - origin.z);
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

  public Vector3D rotate(Vector3D eulerAngles) {
    Vector3D sines = new Vector3D(Math.sin(eulerAngles.x()), Math.sin(eulerAngles.y()), Math.sin(eulerAngles.z()));
    Vector3D cosines = new Vector3D(Math.cos(eulerAngles.x()), Math.cos(eulerAngles.y()), Math.cos(eulerAngles.z()));
    return new Vector3D(this.x * cosines.z * cosines.y +
            this.y * (cosines.z * sines.y * sines.x - sines.z * cosines.x) + this.z * (cosines.z * sines.y * cosines.x + sines.z * sines.x),
            this.x * sines.z * cosines.y +
                    this.y * (cosines.z * cosines.x + sines.z * sines.y * sines.x) + this.z * (-cosines.z * sines.x + sines.z * sines.y * cosines.x),
            -this.x * sines.y + this.y * cosines.y * sines.x + this.z * cosines.y * cosines.x);
  }

  public Vector3D reverseRotate(Vector3D eulerAngles) {
    Vector3D sines = new Vector3D(Math.sin(-eulerAngles.x()), Math.sin(-eulerAngles.y()), Math.sin(-eulerAngles.z()));
    Vector3D cosines = new Vector3D(Math.cos(eulerAngles.x()), Math.cos(eulerAngles.y()), Math.cos(eulerAngles.z()));
    return new Vector3D(
            this.x * cosines.z * cosines.y - this.y * sines.z * cosines.y + this.z * sines.y,
            this.x * (cosines.z * sines.y * sines.x + sines.z * cosines.x) +
                    this.y * (cosines.z * cosines.x - sines.z * sines.y * sines.x) - this.z * cosines.y * sines.x,
            this.x * (-cosines.z * sines.y * cosines.x + sines.z * sines.x) +
                    this.y * (cosines.z * sines.x + sines.z * sines.y * cosines.x) + this.z * cosines.y * cosines.x);
  }
}