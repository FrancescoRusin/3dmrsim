package tasks;

import geometry.Vector3D;
import io.github.ericmedvet.jnb.datastructure.DoubleRange;
import snapshot.InstantSnapshot;

import java.util.*;

public class Outcome {
    protected final SortedMap<Double, InstantSnapshot> observations;
    private final Map<DoubleRange, Outcome> subOutcomeCacher;
    private final Map<Key, Double> resultsCacher;
    private static final int MAX_CACHED_SUBOUTCOMES = 3;

    public Outcome(Map<Double, InstantSnapshot> observations) {
        this.observations = new TreeMap<>(observations);
        this.subOutcomeCacher = new HashMap<>();
        this.resultsCacher = new HashMap<>();
    }

    public double averageDistanceFromTarget(Vector3D target) {
        return get(new Key(Function.SA_DISTANCE_FROM_TARGET, Operator.AVERAGE));
    }

    public double finalDistanceFromTarget(Vector3D target) {
        return get(new Key(Function.SA_DISTANCE_FROM_TARGET, Operator.FINAL));
    }

    public double maxDistanceFromTarget(Vector3D target) {
        return get(new Key(Function.SA_DISTANCE_FROM_TARGET, Operator.MAX));
    }

    public double minDistanceFromTarget(Vector3D target) {
        return get(new Key(Function.SA_DISTANCE_FROM_TARGET, Operator.MIN));
    }

    // not every function makes sense for both singleagent and multiagent cases so singleagent functions have a SA prefix while multiagent have MA
    private enum Function {
        SA_DISTANCE_FROM_TARGET
    }

    private enum Operator {
        AVERAGE, FINAL, MIN, MAX
    }

    private record Key(Function function, Operator operator) {}

    private Double get(InstantSnapshot snapshot, Function function, Object... args) {
        switch (function) {
            case SA_DISTANCE_FROM_TARGET:
                Vector3D target = (Vector3D) args[0];
                return snapshot.activeBodies().get(0).position().vectorDistance(target).norm();
        }
        return 0d;
    }

    private Double get(Key key, Object... args) {
        if (!resultsCacher.containsKey(key)) {
            resultsCacher.put(key, switch (key.operator) {
                case AVERAGE -> observations.values().stream().mapToDouble(s -> get(s, key.function, args)).average().orElse(0d);
                case FINAL -> get(observations.lastEntry().getValue(), key.function, args);
                case MIN -> observations.values().stream().mapToDouble(s -> get(s, key.function, args)).min().orElse(0d);
                case MAX -> observations.values().stream().mapToDouble(s -> get(s, key.function, args)).max().orElse(0d);
            });
        }
        return resultsCacher.get(key);
    }

    public Outcome suboutcome(DoubleRange range) {
        if (!subOutcomeCacher.containsKey(range)) {
            if (subOutcomeCacher.size() >= MAX_CACHED_SUBOUTCOMES) {
                subOutcomeCacher.remove(subOutcomeCacher.keySet().iterator().next());
            }
            subOutcomeCacher.put(range, new Outcome(observations.subMap(range.min(), range.max())));
        }
        return subOutcomeCacher.get(range);
    }
}
