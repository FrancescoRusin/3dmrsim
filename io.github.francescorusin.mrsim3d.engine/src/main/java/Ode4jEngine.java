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
import java.util.List;

import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

public class Ode4jEngine implements Engine {
  private static int initialize = OdeHelper.initODE2(0);
  private DWorld world;
  private DSpace space;
  protected double time;
  protected List<EmbodiedAgent> agents;
  protected List<Body> passiveBodies;

  public Ode4jEngine() {
    world = OdeHelper.createWorld();
    space = OdeHelper.createHashSpace(null);
  }

  public List<EmbodiedAgent> getAgents() {
    return agents;
  }

  private List<Body> getPassiveBodies() {
    return passiveBodies;
  }

  @Override
  public double t() {
    return time;
  }

  @Override
  public Snapshot tick() {
    return null;
  }

  @Override
  public void addBody(Body body) {}
}
