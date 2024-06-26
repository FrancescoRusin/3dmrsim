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
            System.out.printf("Total: %.4f seconds\n", (System.currentTimeMillis() - startTimeMillis) / 1000d);
            System.out.printf("engine: %.4f; signals: %.4f; other: %.4f (%.4f vs %.4f vs %.4f)\n",
                    engine.timeTickEngine / 1000d, engine.timeTickSignals / 1000d, engine.timeTickOther / 1000d,
                    engine.timeTickEngine / totalTimeMillis,
                    engine.timeTickSignals / totalTimeMillis,
                    engine.timeTickOther / totalTimeMillis
            );
            engine.destroy();
        }
    }
}