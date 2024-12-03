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
package test;

import engine.Ode4jEngine;

public class PerformanceTest extends Test {

    public static void main(String[] args) {
        new PerformanceTest().unstaticMain();
    }

    public void unstaticMain() {
        long startTimeMillis;
        double totalTimeMillis;
        int commChannels = 0;
        System.out.printf("Hundred voxels test (%d comm channels)\n", commChannels);
        for (int i = 0; i < 10; ++i) {
            engine = new Ode4jEngine();
            startTimeMillis = System.currentTimeMillis();
            hundredVoxelsTest(commChannels);
            while (engine.t() < 100) {
                engine.tick();
            }
            totalTimeMillis = engine.timeTickEngine + engine.timeTickSignals + engine.timeTickOther;
            System.out.printf(
                    "Total: %.4f seconds\n", (System.currentTimeMillis() - startTimeMillis) / 1000d);
            System.out.printf(
                    "engine: %.4f; signals: %.4f; other: %.4f (%.4f vs %.4f vs %.4f)\n",
                    engine.timeTickEngine / 1000d,
                    engine.timeTickSignals / 1000d,
                    engine.timeTickOther / 1000d,
                    engine.timeTickEngine / totalTimeMillis,
                    engine.timeTickSignals / totalTimeMillis,
                    engine.timeTickOther / totalTimeMillis);
            engine.destroy();
        }
    }
}
