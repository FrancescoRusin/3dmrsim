package tasks;

import engine.Ode4jEngine;
import snapshot.InstantSnapshot;

import java.util.function.Consumer;

public interface Task<A, O> {
    O run(A a, Ode4jEngine engine, Consumer<InstantSnapshot> snapshotConsumer);

    default O run(A a, Ode4jEngine engine) {
        return run(a, engine, s -> {
        });
    }
}
