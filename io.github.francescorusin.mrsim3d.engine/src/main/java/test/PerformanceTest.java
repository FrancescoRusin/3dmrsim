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
        }
    }
}