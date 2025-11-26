package tasks;

import geometry.Vector3D;
import io.github.ericmedvet.jnb.datastructure.DoubleRange;
import snapshot.BodySnapshot;
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
        return get(new Key(AgentSelector.SINGLE_AGENT, Function.DISTANCE_FROM_TARGET, TimeOperator.AVERAGE), target);
    }

    public double finalDistanceFromTarget(Vector3D target) {
        return get(new Key(AgentSelector.SINGLE_AGENT, Function.DISTANCE_FROM_TARGET, TimeOperator.FINAL), target);
    }

    public double maxDistanceFromTarget(Vector3D target) {
        return get(new Key(AgentSelector.SINGLE_AGENT, Function.DISTANCE_FROM_TARGET, TimeOperator.MAX), target);
    }

    public double minDistanceFromTarget(Vector3D target) {
        return get(new Key(AgentSelector.SINGLE_AGENT, Function.DISTANCE_FROM_TARGET, TimeOperator.MIN), target);
    }

    private enum Function {
        DISTANCE_FROM_TARGET
    }

    private enum AgentSelector {
        SINGLE_AGENT, ALL_AGENTS_AVERAGE, ALL_AGENTS_MIN, ALL_AGENTS_MAX
    }

    private enum TimeOperator {
        AVERAGE, FINAL, MIN, MAX
    }

    private record Key(AgentSelector agentSelector, Function function, TimeOperator timeOperator) {}

    private Double get(BodySnapshot body, Function function, Object... args) {
        return switch (function) {
            case DISTANCE_FROM_TARGET -> body.position().vectorDistance((Vector3D) args[0]).norm();
        };
    }

    private Double get(InstantSnapshot snapshot, AgentSelector agentSelector, Function function, Object... args) {
        return switch (agentSelector) {
            case SINGLE_AGENT -> get(snapshot.activeBodies().getFirst(), function, args);
            case ALL_AGENTS_AVERAGE -> snapshot.activeBodies().stream().mapToDouble(b -> get(b, function, args)).average().orElse(0d);
            case ALL_AGENTS_MIN -> snapshot.activeBodies().stream().mapToDouble(b -> get(b, function, args)).min().orElse(0d);
            case ALL_AGENTS_MAX -> snapshot.activeBodies().stream().mapToDouble(b -> get(b, function, args)).max().orElse(0d);
        };
    }

    private Double get(Key key, Object... args) {
        if (!resultsCacher.containsKey(key)) {
            resultsCacher.put(key, switch (key.timeOperator) {
                case AVERAGE -> observations.values().stream().mapToDouble(s -> get(s, key.agentSelector, key.function, args)).average().orElse(0d);
                case FINAL -> get(observations.lastEntry().getValue(), key.agentSelector, key.function, args);
                case MIN -> observations.values().stream().mapToDouble(s -> get(s, key.agentSelector, key.function, args)).min().orElse(0d);
                case MAX -> observations.values().stream().mapToDouble(s -> get(s, key.agentSelector, key.function, args)).max().orElse(0d);
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
