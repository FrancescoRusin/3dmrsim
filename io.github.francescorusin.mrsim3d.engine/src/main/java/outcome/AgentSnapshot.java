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
package outcome;

import geometry.Vector3D;

import java.util.List;

//TEMPORARY! JUST FOR TESTING!
public interface AgentSnapshot extends ObjectSnapshot {
    List<ObjectSnapshot> components();

    @Override
    default Vector3D position() {
        List<ObjectSnapshot> components = components();
        return components.stream()
                .map(ObjectSnapshot::position)
                .reduce(Vector3D::sum)
                .orElse(new Vector3D())
                .times(1d / components.size());
    }
}
