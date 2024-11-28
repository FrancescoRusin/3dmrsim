package outcome; /*-
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

import actions.Action;
import bodies.Body;
import geometry.Vector3D;
import java.util.List;
import java.util.Map;
import utils.Pair;
import utils.UnorderedPair;

public record InstantSnapshot(
    Map<Integer, AgentSnapshot> agents,
    Map<Integer, ObjectSnapshot> passiveBodies,
    List<Action> actions,
    Map<UnorderedPair<Body>, List<Pair<Vector3D, Vector3D>>> springJoints,
    Map<UnorderedPair<Body>, List<Pair<Vector3D, Vector3D>>> fixedJoints) {}
