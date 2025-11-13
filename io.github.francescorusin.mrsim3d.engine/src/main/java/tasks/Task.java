package tasks;

import snapshot.InstantSnapshot;

import java.util.function.Consumer;

public interface Task<A, O> {
    O run(A a, Consumer<InstantSnapshot> snapshotConsumer);

    default O run(A a) {
        return run(a, s -> {
        });
    }
}
