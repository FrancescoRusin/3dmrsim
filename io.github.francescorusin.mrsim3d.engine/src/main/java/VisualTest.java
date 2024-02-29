/*-
 * ========================LICENSE_START=================================
 * engine
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

import static drawstuff.DrawStuff.*;

import agents.EmbodiedAgent;
import agents.SingleVoxelAgent;
import bodies.Body;
import bodies.Sphere;
import bodies.Voxel;
import drawstuff.DrawStuff;
import engine.Ode4jEngine;
import geometry.Vector3D;
import java.util.EnumMap;
import java.util.EnumSet;
import org.ode4j.ode.*;

public class VisualTest extends DrawStuff.dsFunctions {
  private static final float[] xyz = {0f, -4f, 1.7600f};
  private static final float[] hpr = {90f, -10f, 0f};
  private static final double TEST_SIDE_LENGTH = 1d;
  private static final double TEST_SPHERE_RADIUS_RATIO = 0.15;
  private static final double TEST_MASS = 1d;
  private static final double TEST_SIDE_LENGTH_STRETCH_RATIO = .3;
  private final Ode4jEngine engine = new Ode4jEngine();
  private final SingleVoxelAgent voxel1 =
      new SingleVoxelAgent(
          TEST_SIDE_LENGTH * (1d - 2d * TEST_SPHERE_RADIUS_RATIO),
          TEST_SIDE_LENGTH * TEST_SPHERE_RADIUS_RATIO,
          TEST_MASS,
          100,
          0,
          TEST_SIDE_LENGTH_STRETCH_RATIO,
          EnumSet.of(Voxel.JointOption.EDGES, Voxel.JointOption.SIDES, Voxel.JointOption.INTERNAL),
          "a-v",
          d -> {
            EnumMap<Voxel.Edge, Double> testControllerInput = new EnumMap<>(Voxel.Edge.class);
            Voxel.Edge[] edgeValues = Voxel.Edge.values();
            for (int i = 0; i < 4; ++i) {
              testControllerInput.put(edgeValues[i], Math.sin(d));
            }
            for (int i = 4; i < 12; ++i) {
              testControllerInput.put(edgeValues[i], 0d);
            }
            return testControllerInput;
          });
  private final SingleVoxelAgent voxel2 =
      new SingleVoxelAgent(
          TEST_SIDE_LENGTH * (1d - 2d * TEST_SPHERE_RADIUS_RATIO),
          TEST_SIDE_LENGTH * TEST_SPHERE_RADIUS_RATIO,
          TEST_MASS,
          100,
          0,
          TEST_SIDE_LENGTH_STRETCH_RATIO,
          EnumSet.of(Voxel.JointOption.EDGES, Voxel.JointOption.SIDES, Voxel.JointOption.INTERNAL),
          "a-v",
          d -> {
            EnumMap<Voxel.Edge, Double> testControllerInput = new EnumMap<>(Voxel.Edge.class);
            Voxel.Edge[] edgeValues = Voxel.Edge.values();
            for (int i = 0; i < 4; ++i) {
              testControllerInput.put(edgeValues[i], 0d);
              testControllerInput.put(edgeValues[i + 8], 0d);
            }
            for (int i = 4; i < 8; ++i) {
              testControllerInput.put(edgeValues[i], Math.sin(d));
            }
            return testControllerInput;
          });
  private final SingleVoxelAgent voxel3 =
      new SingleVoxelAgent(
          TEST_SIDE_LENGTH * (1d - 2d * TEST_SPHERE_RADIUS_RATIO),
          TEST_SIDE_LENGTH * TEST_SPHERE_RADIUS_RATIO,
          TEST_MASS,
          100,
          0,
          TEST_SIDE_LENGTH_STRETCH_RATIO,
          EnumSet.of(Voxel.JointOption.EDGES, Voxel.JointOption.SIDES, Voxel.JointOption.INTERNAL),
          "a-v",
          d -> {
            EnumMap<Voxel.Edge, Double> testControllerInput = new EnumMap<>(Voxel.Edge.class);
            Voxel.Edge[] edgeValues = Voxel.Edge.values();
            for (int i = 0; i < 8; ++i) {
              testControllerInput.put(edgeValues[i], 0d);
            }
            for (int i = 8; i < 12; ++i) {
              testControllerInput.put(edgeValues[i], Math.sin(d));
            }
            return testControllerInput;
          });

  public static void main(String[] args) {
    new VisualTest().demo(args);
  }

  public void demo(String[] args) {
    engine.addAgent(voxel1, new Vector3D(-1d, -1d, 1d));
    engine.addAgent(voxel2, new Vector3D(1d, -1d, 1d));
    engine.addAgent(voxel3, new Vector3D(0d, 1d, 1d));
    dsSimulationLoop(args, 1080, 720, this);
    engine.getSpace().destroy();
    engine.getWorld().destroy();
    OdeHelper.closeODE();
  }

  @Override
  public void start() {
    dsSetViewpoint(xyz, hpr);
  }

  @Override
  public void step(boolean pause) {
    if (!pause) {
      engine.tick();
    }

    dsSetColor(1, 1, 0);
    dsSetTexture(DS_TEXTURE_NUMBER.DS_WOOD);
    for (EmbodiedAgent agent : engine.getAgents()) {
      for (Body body : ((Voxel) agent).bodyParts())
        dsDrawSphere(
            body.getBody().getPosition(),
            body.getBody().getRotation(),
            ((Sphere) body).getRadius());
    }
    dsSetColor(0, 0, 0);
    dsSetTexture(DS_TEXTURE_NUMBER.DS_CHECKERED);
    for (DJoint joint : engine.softJoints.values()) {
      if (joint instanceof DDoubleBallJoint doubleBallJoint) {
        dsDrawLine(
            doubleBallJoint.getBody(0).getPosition(), doubleBallJoint.getBody(1).getPosition());
      }
    }
    dsSetColor(1, 1, 1);
    for (DJoint joint : engine.rigidJoints.values()) {
      if (joint instanceof DDoubleBallJoint doubleBallJoint) {
        dsDrawLine(
            doubleBallJoint.getBody(0).getPosition(), doubleBallJoint.getBody(1).getPosition());
      }
    }
  }

  @Override
  public void command(char cmd) {}

  @Override
  public void stop() {}
}
