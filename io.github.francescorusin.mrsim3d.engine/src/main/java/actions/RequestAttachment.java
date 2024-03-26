package actions;

import ad.Attachable;
import bodies.Body;
import bodies.Voxel;
import engine.Ode4jEngine;
import geometry.Vector3D;
import utils.Pair;

import java.util.*;

public class RequestAttachment implements Action {
    private final Attachable requester;
    private final List<Body> requesterAttachGroup;
    private final double springConstant;
    private final double dampingConstant;


    public RequestAttachment(Attachable requester, List<Body> requesterAttachGroup,
                             double springConstant, double dampingConstant) {
        this.requester = requester;
        this.requesterAttachGroup = requesterAttachGroup;
        this.springConstant = springConstant;
        this.dampingConstant = dampingConstant;
    }

    public RequestAttachment(Attachable requester, List<Body> requesterAttachGroup) {
        this(requester, requesterAttachGroup,
                Voxel.DEFAULT_SPRING_CONSTANT, Voxel.DEFAULT_DAMPING_CONSTANT);
    }

    @Override
    public void execute(Ode4jEngine engine) {
        //TODO DEBUG
        Vector3D basePos = requesterAttachGroup.stream()
                .map(b -> b.position(engine.t()).times(b.mass())).reduce(Vector3D::sum).orElseThrow()
                .times(1d / requesterAttachGroup.stream().mapToDouble(Body::mass).sum());
        Vector3D planeNormal = Vector3D.weirdNormalApproximation(
                requesterAttachGroup.stream().map(v -> v.position(engine.t()).vectorDistance(basePos)).toList());
        double planeC = basePos.scalarProduct(planeNormal);
        double correctSign = -Math.signum(requester.position(engine.t()).scalarProduct(planeNormal) - planeC);
        Vector3D maxBasePos = basePos.sum(new Vector3D(
                engine.configuration.maxAttractDistance(),
                engine.configuration.maxAttractDistance(),
                engine.configuration.maxAttractDistance())
        );
        Vector3D minBasePos = basePos.sum(new Vector3D(
                -engine.configuration.maxAttractDistance(),
                -engine.configuration.maxAttractDistance(),
                -engine.configuration.maxAttractDistance())
        );
        // cull attachment possibilities by filtering out anything that is not in range by using cached bounding box
        Attachable closestAttachable = engine.allObjectsStream()
                .filter(sb -> sb.boundingBox(engine.t()).min().x() < maxBasePos.x() &&
                        sb.boundingBox(engine.t()).max().x() > minBasePos.x() &&
                        sb.boundingBox(engine.t()).min().y() < maxBasePos.y() &&
                        sb.boundingBox(engine.t()).max().y() > minBasePos.y() &&
                        sb.boundingBox(engine.t()).min().z() < maxBasePos.z() &&
                        sb.boundingBox(engine.t()).max().z() > minBasePos.z()
                )
                .filter(sb -> sb instanceof Attachable && sb != requester &&
                        correctSign * (sb.position(engine.t()).scalarProduct(planeNormal) - planeC) > 0)
                .map(sb -> (Attachable) sb).min(Comparator.comparingDouble(v -> v.position(engine.t()).vectorDistance(basePos).norm()))
                .orElse(null);
        if (Objects.isNull(closestAttachable) ||
                closestAttachable.position(engine.t()).vectorDistance(basePos).norm() > engine.configuration.maxAttractDistance()) {
            return;
        }
        Map<List<Body>, Vector3D> possibilitiesPositions = new HashMap<>();
        Map<List<Body>, Double> possibilitiesDistances = new HashMap<>();
        for (List<Body> attachPossibility : closestAttachable.attachPossibilities()) {
            possibilitiesPositions.put(attachPossibility, attachPossibility.stream()
                    .map(b -> b.position(engine.t()).times(b.mass())).reduce(Vector3D::sum).orElseThrow()
                    .times(1d / attachPossibility.stream().mapToDouble(Body::mass).sum()));
            possibilitiesDistances.put(attachPossibility,
                    possibilitiesPositions.get(attachPossibility).vectorDistance(basePos).norm()
            );
        }
        List<Body> bestAnchorBlock = Collections.min(possibilitiesDistances.keySet(), Comparator.comparingDouble(possibilitiesDistances::get));
        Map<Pair<Body, Body>, Double> bodyDistances = new HashMap<>();
        for (Body b1 : requesterAttachGroup) {
            for (Body b2 : bestAnchorBlock) {
                bodyDistances.put(new Pair<>(b1, b2), b1.position(engine.t()).vectorDistance(b2.position(engine.t())).norm());
            }
        }
        Pair<Body, Body> minDistanceBodies = Collections.min(bodyDistances.keySet(), Comparator.comparingDouble(bodyDistances::get));
        if (bodyDistances.get(minDistanceBodies) > engine.configuration.maxAttachDistance()) {
            Vector3D force = possibilitiesPositions.get(bestAnchorBlock).vectorDistance(basePos).normalize()
                    .times(engine.configuration.attractForceModule());
            for (Body b1 : requesterAttachGroup) {
                b1.dBody().addForce(force.x(), force.y(), force.z());
            }
            for (Body b2 : bestAnchorBlock) {
                b2.dBody().addForce(-force.x(), -force.y(), -force.z());
            }
            return;
        }
        if (!minDistanceBodies.first().dBody().isConnectedTo(minDistanceBodies.second().dBody())) {
            engine.addSpringJoint(minDistanceBodies.first(), minDistanceBodies.second(), springConstant, dampingConstant)
                    .setDistance(engine.configuration.attachSpringRestDistance());
        }
        List<Pair<Body, Body>> sortedBodyPairs = new ArrayList<>(
                bodyDistances.keySet().stream().sorted(Comparator.comparing(bodyDistances::get).reversed()).toList()
        );
        sortedBodyPairs.removeIf(bp -> bp.first() == minDistanceBodies.first() || bp.second() == minDistanceBodies.second());
        while (!sortedBodyPairs.isEmpty()) {
            Pair<Body, Body> newMinDistanceBodies = sortedBodyPairs.remove(sortedBodyPairs.size() - 1);
            if (bodyDistances.get(newMinDistanceBodies) > engine.configuration.maxAttachDistance()) {
                if (bodyDistances.get(newMinDistanceBodies) > engine.configuration.maxAttractDistance()) {
                    break;
                }
                Vector3D force = newMinDistanceBodies.second().position(engine.t())
                        .vectorDistance(newMinDistanceBodies.first().position(engine.t()));
                force = force.times(bodyDistances.get(newMinDistanceBodies) / engine.configuration.maxAttractDistance());
                newMinDistanceBodies.first().dBody().addForce(force.x(), force.y(), force.z());
                newMinDistanceBodies.second().dBody().addForce(-force.x(), -force.y(), -force.z());
            }
            if (!requester.attachedBodies().get(newMinDistanceBodies.first()).contains(newMinDistanceBodies.second())) {
                engine.addSpringJoint(newMinDistanceBodies.first(), newMinDistanceBodies.second(), springConstant, dampingConstant)
                        .setDistance(engine.configuration.attachSpringRestDistance());
                requester.attachedBodies().get(newMinDistanceBodies.first()).add(newMinDistanceBodies.second());
            }
            sortedBodyPairs.removeIf(bp -> bp.first() == newMinDistanceBodies.first() || bp.second() == newMinDistanceBodies.second());
        }
    }
}