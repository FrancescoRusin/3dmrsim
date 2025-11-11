package tasks;

import agents.EmbodiedAgent;
import engine.Ode4jEngine;
import snapshot.InstantSnapshot;
import terrains.Terrain;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class Locomotion implements Task<Supplier<EmbodiedAgent>, Outcome> {
    private final Terrain terrain;

    public Locomotion(Terrain terrain) {
        this.terrain = terrain;
    }

    //TODO

    @Override
    public Outcome run(Supplier<EmbodiedAgent> embodiedAgentSupplier, Ode4jEngine engine, Consumer<InstantSnapshot> snapshotConsumer) {
        return null;
    }
}
