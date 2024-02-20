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
import java.util.*;
import org.ode4j.ode.*;

public class Ode4jEngine extends AbstractEngine {
  private static int initialize = OdeHelper.initODE2(0);
  private final DWorld world;
  private final DSpace space;
  protected LinkedHashMap<EmbodiedAgent, List<DBody>> agents = new LinkedHashMap<>();
  protected LinkedHashMap<Body, DBody> passiveBodies = new LinkedHashMap<>();
  protected DGeom terrain;

  public Ode4jEngine() {
    world = OdeHelper.createWorld();
    space = OdeHelper.createHashSpace(null);
    world.setGravity(DEFAULT_GRAVITY.x(), DEFAULT_GRAVITY.y(), DEFAULT_GRAVITY.z());
    // TODO ADD TERRAINS
    terrain = OdeHelper.createPlane(space, 0, 0, 1, 0);
  }

  public Set<EmbodiedAgent> getAgents() {
    return agents.keySet();
  }

  private Set<Body> getPassiveBodies() {
    return passiveBodies.keySet();
  }

  @Override
  public double t() {
    return time;
  }

  @Override
  public Snapshot tick() {
    return null;
  }

  public DWorld getWorld() {
    return world;
  }

  @Override
  public void addBody(Body body) {
  }
}
