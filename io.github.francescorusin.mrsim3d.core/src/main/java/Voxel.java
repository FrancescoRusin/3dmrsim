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

public interface Voxel extends MultiBody {
    public enum Vertex {
        V000,
        V001,
        V010,
        V011,
        V100,
        V101,
        V110,
        V111
    }

    enum Side {
        UP(Vertex.V001, Vertex.V101, Vertex.V111, Vertex.V011),
        DOWN(Vertex.V000, Vertex.V010, Vertex.V110, Vertex.V100),
        FRONT(Vertex.V010, Vertex.V011, Vertex.V111, Vertex.V110),
        BACK(Vertex.V000, Vertex.V100, Vertex.V101, Vertex.V001),
        RIGHT(Vertex.V100, Vertex.V110, Vertex.V111, Vertex.V101),
        LEFT(Vertex.V000, Vertex.V001, Vertex.V011, Vertex.V010);
        private final Vertex v1;
        private final Vertex v2;
        private final Vertex v3;
        private final Vertex v4;

        Side(Vertex v1, Vertex v2, Vertex v3, Vertex v4) {
            this.v1 = v1;
            this.v2 = v2;
            this.v3 = v3;
            this.v4 = v4;
        }
    }

    public enum JointOption {
        INTERAL,
        SIDES,
        EDGES
    }
}
