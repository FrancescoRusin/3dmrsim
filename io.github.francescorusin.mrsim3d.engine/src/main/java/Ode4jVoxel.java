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
import geometry.Vector3D;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;
import org.ode4j.ode.DBallJoint;
import utils.Pair;

public class Ode4jVoxel implements Voxel {

  EnumSet<JointOption> jointOptions;
  Map<Vertex, Body> rigidBodies;
  Map<Pair<Vertex, Vertex>, DBallJoint> joints;
  List<Body> ulteriorBodies;
  double sideLength;
  double mass;
  double friction;
  double springConstant;
  double dampingConstant;
  double rigidMassLengthRatio;
  double[] areaRatio;

  public Ode4jVoxel(
      double sideLength,
      double mass,
      double friction,
      double springConstant,
      double dampingConstant,
      double rigidMassLengthRatio,
      double minAreaRatio,
      double maxAreaRatio,
      EnumSet<JointOption> jointOptions) {
    this.sideLength = sideLength;
    this.mass = mass;
    this.friction = friction;
    this.springConstant = springConstant;
    this.dampingConstant = dampingConstant;
    this.rigidMassLengthRatio = rigidMassLengthRatio;
    this.areaRatio = new double[] {minAreaRatio > 0 ? minAreaRatio : 0, maxAreaRatio};
    this.jointOptions = jointOptions;
  }

  @Override
  public List<Body> getComponents() {
    return Stream.concat(rigidBodies.values().stream(), ulteriorBodies.stream())
        .map(b -> (Body) b)
        .toList();
  }

  @Override
  public double volume() {
    return 0;
  }

  @Override
  public void assemble(Engine engine, Vector3D position) {}
}
