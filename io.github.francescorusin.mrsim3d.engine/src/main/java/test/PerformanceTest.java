package test;

import engine.Ode4jEngine;

public class PerformanceTest extends VisualTest {

    public static void main(String[] args) {
            new PerformanceTest().unstaticMain();
    }

    public void unstaticMain() {
        long startTimeMillis;
        for (int i = 0; i < 10; ++i) {
            engine = new Ode4jEngine();
            startTimeMillis = System.currentTimeMillis();
            hundredVoxelsTest();
            while (engine.t() < 100) {
                engine.tick();
            }
            System.out.printf("Hundred voxels test result: %.4f seconds\n", (System.currentTimeMillis() - startTimeMillis) / 1000d);
            System.out.printf("Time spent on engine: %.4f; time spent on other: %.4f (%.4f vs %.4f)\n",
                    engine.timeTickEngine / 1000d, engine.timeTickOther / 1000d,
                    engine.timeTickEngine / (double) (engine.timeTickEngine + engine.timeTickOther),
                    engine.timeTickOther / (double) (engine.timeTickEngine + engine.timeTickOther)
            );
        }
    }
}