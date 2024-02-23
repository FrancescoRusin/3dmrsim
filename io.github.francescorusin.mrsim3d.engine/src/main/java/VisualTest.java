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

import agents.EmbodiedAgent;
import bodies.Body;
import bodies.Sphere;
import bodies.Voxel;
import drawstuff.DrawStuff;
import engine.Ode4jEngine;
import geometry.Vector3D;
import org.ode4j.ode.*;
import org.ode4j.ode.internal.DxSphere;

import java.util.EnumSet;
import java.util.List;

import static drawstuff.DrawStuff.*;

public class VisualTest extends DrawStuff.dsFunctions {
  private static float[] xyz = {2.1640f, -1.3079f, 1.7600f};
  private static float[] hpr = {125.5000f, -17.0000f, 0.0000f};
  private Ode4jEngine engine = new Ode4jEngine();
  private Voxel voxel = new Voxel(1, 1, 1, 5, 0, .3, .6, 1.4,
          EnumSet.of(Voxel.JointOption.EDGES, Voxel.JointOption.SIDES, Voxel.JointOption.INTERAL));

  public static void main(String[] args) {
    new VisualTest().demo(args);
  }

  public void demo(String[] args) {
    engine.addAgent(voxel, new Vector3D(0d, 0d, 1d));
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
        dsDrawSphere(body.getBody().getPosition(), body.getBody().getRotation(), ((Sphere) body).getRadius());
    }
  }

  @Override
  public void command(char cmd) {
  }

  @Override
  public void stop() {
  }
}
