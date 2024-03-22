package actions;

import ad.Attachable;
import bodies.Body;
import bodies.Voxel;
import engine.Ode4jEngine;
import geometry.Vector3D;
import org.ode4j.ode.DDoubleBallJoint;
import utils.Pair;

import java.util.*;

public class RequestAttachment implements Action {
    private final List<Body> requesters;
    private final double maxAttachDistance;
    private final double maxAttractDistance;
    private final double springConstant;
    private final double dampingConstant;
    private final double springRestDistance;


    public RequestAttachment(List<Body> requesters, double maxAttachDistance, double maxAttractDistance,
                             double springConstant, double dampingConstant, double springRestDistance) {
        this.requesters = requesters;
        this.maxAttachDistance = maxAttachDistance;
        this.maxAttractDistance = maxAttractDistance;
        this.springConstant = springConstant;
        this.dampingConstant = dampingConstant;
        this.springRestDistance = springRestDistance;
    }

    public RequestAttachment(List<Body> requesters) {
        this(requesters, Attachable.DEFAULT_ATTACH_DISTANCE, Attachable.DEFAULT_ATTRACT_DISTANCE,
                Voxel.DEFAULT_SPRING_CONSTANT, Voxel.DEFAULT_DAMPING_CONSTANT, Attachable.DEFAULT_ATTACH_DISTANCE / 2);
    }

    @Override
    public void execute(Ode4jEngine engine) {
        //TODO TEST
        Pair<Vector3D, Double> basePosAndMass = requesters.stream().map(b -> new Pair<>(b.position(engine.t()), b.mass()))
                .reduce((p1, p2) -> new Pair<>(p1.first().weightedSum(p2.first(), p1.second(), p2.second()), p1.second() + p2.second()))
                .orElseThrow();
        Vector3D basePos = basePosAndMass.first().times(1d / basePosAndMass.second());
        Vector3D maxBasePos = basePos.sum(new Vector3D(maxAttachDistance, maxAttachDistance, maxAttachDistance));
        Vector3D minBasePos = basePos.sum(new Vector3D(-maxAttachDistance, -maxAttachDistance, -maxAttachDistance));
        // cut out attachment possibilities by filtering out anything that is not in range by using cached bounding box
        Attachable closestAttachable = (Attachable) engine.allBodiesStream().filter(sb -> sb instanceof Attachable)
                .filter(sb ->
                        sb.boundingBox(engine.t()).min().x() < maxBasePos.x() &&
                                sb.boundingBox(engine.t()).max().x() > minBasePos.x() &&
                                sb.boundingBox(engine.t()).min().y() < maxBasePos.y() &&
                                sb.boundingBox(engine.t()).max().y() > minBasePos.y() &&
                                sb.boundingBox(engine.t()).min().z() < maxBasePos.z() &&
                                sb.boundingBox(engine.t()).max().z() > minBasePos.z()
                ).min(Comparator.comparingDouble(v -> v.position(engine.t()).vectorDistance(basePos).norm()))
                .orElse(null);
        if (Objects.isNull(closestAttachable) || closestAttachable.position(engine.t()).vectorDistance(basePos).norm() > maxAttractDistance) {
            return;
        }
        Map<List<Body>, Double> possibilitiesDistances = new HashMap<>();
        for (List<Body> attachPossibility : closestAttachable.attachPossibilities()) {
            possibilitiesDistances.put(attachPossibility, attachPossibility.stream().map(b -> new Pair<>(b.position(engine.t()), b.mass()))
                    .reduce((p1, p2) -> new Pair<>(p1.first().weightedSum(p2.first(), p1.second(), p2.second()), p1.second() + p2.second()))
                    .orElseThrow().second());
        }
        List<Body> bestAnchorBlock = Collections.max(possibilitiesDistances.keySet(), Comparator.comparingDouble(possibilitiesDistances::get));
        Map<Pair<Body, Body>, Double> bodyDistances = new HashMap<>();
        for (Body b1 : requesters) {
            for (Body b2 : bestAnchorBlock) {
                bodyDistances.put(new Pair<>(b1, b2), b1.position(engine.t()).vectorDistance(b2.position(engine.t())).norm());
            }
        }
        Pair<Body, Body> minDistanceBodies = Collections.min(bodyDistances.keySet(), Comparator.comparingDouble(bodyDistances::get));
        if (bodyDistances.get(minDistanceBodies) > maxAttachDistance) {
            //TODO FORCES
            return;
        }
        if (!minDistanceBodies.first().dBody().isConnectedTo(minDistanceBodies.second().dBody())) {
            engine.addSpringJoint(minDistanceBodies.first(), minDistanceBodies.second(), springConstant, dampingConstant)
                    .setDistance(springRestDistance);
        }
        List<Pair<Body, Body>> sortedBodyPairs = new ArrayList<>(
                bodyDistances.keySet().stream().sorted(Comparator.comparing(bodyDistances::get).reversed()).toList()
        );
        sortedBodyPairs.removeIf(bp -> bp.first() == minDistanceBodies.first() || bp.second() == minDistanceBodies.second());
        while (!sortedBodyPairs.isEmpty()) {
            Pair<Body, Body> newMinDistanceBodies = sortedBodyPairs.remove(sortedBodyPairs.size() - 1);
            if (bodyDistances.get(newMinDistanceBodies) > maxAttachDistance) {
                break;
            }
            if (!newMinDistanceBodies.first().dBody().isConnectedTo(newMinDistanceBodies.second().dBody())) {
                engine.addSpringJoint(newMinDistanceBodies.first(), newMinDistanceBodies.second(), springConstant, dampingConstant)
                        .setDistance(springRestDistance);
            }
            sortedBodyPairs.removeIf(bp -> bp.first() == newMinDistanceBodies.first() || bp.second() == newMinDistanceBodies.second());
        }
    }
}