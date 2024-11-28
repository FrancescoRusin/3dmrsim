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
package actions;

import ad.Attachable;
import bodies.Body;
import engine.Ode4jEngine;
import java.util.*;
import viewer.Viewer;

public class RequestDetachment implements Action {
  private final Attachable requester;
  private final List<Body> requesterAttachGroup;

  public RequestDetachment(Attachable requester, List<Body> requesterAttachGroup) {
    this.requester = requester;
    this.requesterAttachGroup = requesterAttachGroup;
  }

  @Override
  public void execute(Ode4jEngine engine) {
    Map<Body, Set<Body>> requesterAttachedBodies = requester.attachedBodies();
    for (Body requesterBody : requesterAttachGroup) {
      for (Body attachedBody : requesterAttachedBodies.get(requesterBody)) {
        engine.removeSpringJoints(requesterBody, attachedBody);
        if (attachedBody instanceof Attachable attachableBody) {
          attachableBody.attachedBodies().get(attachedBody).remove(requesterBody);
        } else if (engine.componentToAgentMapper.get(attachedBody)
            instanceof Attachable attachableAgent) {
          attachableAgent.attachedBodies().get(attachedBody).remove(requesterBody);
        }
      }
      requesterAttachedBodies.get(requesterBody).clear();
    }
  }

  @Override
  public void draw(Viewer viewer) {
    // TODO IMPLEMENT
  }
}
