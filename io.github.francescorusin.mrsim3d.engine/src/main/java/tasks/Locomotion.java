package tasks;

import agents.EmbodiedAgent;
import engine.Ode4jEngine;
import geometry.Vector3D;
import snapshot.InstantSnapshot;
import terrains.FlatTerrain;
import terrains.Terrain;

import java.util.SortedMap;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Locomotion implements Task<Supplier<EmbodiedAgent>, Outcome> {
    private final Terrain terrain;
    private final double duration;
    private final Vector3D initialPosition;

    public Locomotion(Terrain terrain, double duration, Vector3D initialPosition) {
        this.terrain = terrain;
        this.duration = duration;
        this.initialPosition = initialPosition;
    }

    public Locomotion(double duration) {
        this(new FlatTerrain(), duration, new Vector3D(0, 0, 1));
    }

    static int runCounter = 0;
    @Override
    public Outcome run(Supplier<EmbodiedAgent> embodiedAgentSupplier, Ode4jEngine.Mode mode, Consumer<InstantSnapshot> snapshotConsumer) {
        Ode4jEngine engine = new Ode4jEngine(new Ode4jEngine.Configuration(terrain, mode));
        EmbodiedAgent agent = embodiedAgentSupplier.get();
        engine.addAgent(agent, new Vector3D(0, 0, 0));
        // align the agent so that its lowest point has height initialPosition.z
        agent.translate(engine, new Vector3D(initialPosition.x(), initialPosition.y(), initialPosition.z() - agent.boundingBox(0).min().z()));
        SortedMap<Double, InstantSnapshot> observations = new TreeMap<>();
        InstantSnapshot state = engine.currentState();
        observations.put(0d, state);
        snapshotConsumer.accept(state);
        while (engine.t() < duration) {
            state = engine.tick();
            observations.put(engine.t(), state);
            snapshotConsumer.accept(state);
        }
        System.out.printf("RUN %d DONE!\n", runCounter++);
        return new Outcome(observations);
    }
}
