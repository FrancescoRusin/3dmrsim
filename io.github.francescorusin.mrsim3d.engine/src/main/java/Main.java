import agents.CentralizedGridRobot;
import agents.EmbodiedAgent;
import bodies.Voxel;
import geometry.Vector3D;
import io.github.ericmedvet.jgea.core.operator.Crossover;
import io.github.ericmedvet.jgea.core.problem.TotalOrderQualityBasedProblem;
import io.github.ericmedvet.jgea.core.representation.sequence.FixedLengthListFactory;
import io.github.ericmedvet.jgea.core.representation.sequence.numeric.GaussianMutation;
import io.github.ericmedvet.jgea.core.representation.sequence.numeric.SegmentGeometricCrossover;
import io.github.ericmedvet.jgea.core.representation.sequence.numeric.UniformDoubleFactory;
import io.github.ericmedvet.jgea.core.selector.Last;
import io.github.ericmedvet.jgea.core.selector.Tournament;
import io.github.ericmedvet.jgea.core.solver.StandardEvolver;
import io.github.ericmedvet.jsdynsym.core.composed.OutStepped;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import io.github.ericmedvet.jsdynsym.core.numerical.ann.MultiLayerPerceptron;
import tasks.Locomotion;
import tasks.Outcome;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Function;
import java.util.function.Supplier;

public class Main {
    public static void main(String[] args) {
        final int nPop = Integer.parseInt(args[1]);
        final int nEval = Integer.parseInt(args[2]);
        StandardEvolver<List<Double>, List<Double>, Outcome> solver = new StandardEvolver<>(
                g -> g,
                new FixedLengthListFactory<>(33696, new UniformDoubleFactory(-1, 1)),
                nPop,
                s -> s.nOfQualityEvaluations() >= nEval,
                Map.ofEntries(
                        Map.entry(new GaussianMutation(0.35), 0.2),
                        Map.entry(Crossover.from(new SegmentGeometricCrossover().andThen(new GaussianMutation(0.35))), 0.8)
                ),
                new Tournament(Math.max(3, (int) Math.ceil((double) nPop * 0.05))),
                new Last(),
                nPop,
                true,
                100,
                false
        );
        Locomotion task = new Locomotion(30);
        final List<Double> solution;
        try {
            ExecutorService executor = Executors.newFixedThreadPool(36);
            solution = solver.solve(
                    new TotalOrderQualityBasedProblem<>() {
                        @Override
                        public Comparator<Outcome> totalOrderComparator() {
                            return Comparator.comparingDouble(o -> -o.finalDistanceFromTarget(new Vector3D()));
                        }

                        @Override
                        public Function<List<Double>, Outcome> qualityFunction() {
                            return g -> task.run(() -> robot(g));
                        }
                    },
                    new Random(Integer.parseInt(args[0])),
                    executor
            ).stream().findFirst().orElseThrow();
            executor.shutdown();
            BufferedWriter writer = new BufferedWriter(new FileWriter("base-exp.txt", true));
            writer.write("%s\n".formatted(solution));
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static CentralizedGridRobot robot(List<Double> genotype) {
        MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                MultiLayerPerceptron.ActivationFunction.TANH,
                80,
                new int[]{80},
                336
        );
        mlp.setParams(genotype.stream().mapToDouble(d -> d).toArray());
        return new CentralizedGridRobot(
                        testGrid("biped"),
                        NumericalDynamicalSystem.from(
                                new OutStepped<>(mlp, 0.1),
                                80,
                                336
                        )
                );
    }

    private static Voxel[][][] testGrid(String shape) {
        return switch (shape) {
            case "biped" -> {
                Voxel[][][] grid = new Voxel[4][3][3];
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 3; j++) {
                        for (int k = 1; k < 3; k++) {
                            grid[i][j][k] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "");
                        }
                    }
                }
                grid[0][0][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "ang-vlm-vlc-scr-cnt");
                grid[3][0][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "ang-vlm-vlc-scr-cnt");
                grid[0][2][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "ang-vlm-vlc-scr-cnt");
                grid[3][2][0] = new Voxel(EnumSet.allOf(Voxel.JointOption.class), "ang-vlm-vlc-scr-cnt");
                yield grid;
            }
            default -> null;
        };
    }
}