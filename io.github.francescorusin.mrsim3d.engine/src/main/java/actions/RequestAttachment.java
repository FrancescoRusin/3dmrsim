/*-
 * ========================LICENSE_START=================================
 * mrsim3d.engine
 * %%
 * Copyright (C) 2024 Francesco Rusin
 * %%
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * =========================LICENSE_END==================================
 */
package actions;

import ad.Attachable;
import bodies.Body;
import engine.Ode4jEngine;
import geometry.Vector3D;
import utils.Pair;
import viewer.Viewer;

import java.util.*;

public class RequestAttachment implements Action {
    private final Attachable requester;
    private final List<Body> requesterAttachGroup;

    public RequestAttachment(Attachable requester, List<Body> requesterAttachGroup) {
        this.requester = requester;
        this.requesterAttachGroup = requesterAttachGroup;
    }

    @Override
    public void execute(Ode4jEngine engine) {
        if (requesterAttachGroup.stream()
                .map(b -> requester.attachedBodies().get(b))
                .noneMatch(Set::isEmpty)) {
            return;
        }
        Vector3D basePos = requester.attachPossibilitiesPositions(engine.t()).get(requesterAttachGroup);
        Vector3D planeNormal =
                Vector3D.weirdNormalApproximation(
                        requesterAttachGroup.stream()
                                .map(v -> v.position(engine.t()).vectorDistance(basePos))
                                .toList());
        double planeC = basePos.scalarProduct(planeNormal);
        double correctSign =
                -Math.signum(requester.position(engine.t()).scalarProduct(planeNormal) - planeC);
        Attachable closestAttachable =
                engine
                        .allObjectsStream()
                        .filter(
                                sb ->
                                        sb instanceof Attachable
                                                && sb != requester
                                                && correctSign
                                                * (sb.position(engine.t()).scalarProduct(planeNormal) - planeC)
                                                > 0)
                        .map(sb -> (Attachable) sb)
                        .min(
                                Comparator.comparingDouble(
                                        v -> v.position(engine.t()).vectorDistance(basePos).norm()))
                        .orElse(null);
        if (Objects.isNull(closestAttachable)
                || closestAttachable.position(engine.t()).vectorDistance(basePos).norm()
                > engine.configuration.maxAttractDistance()) {
            return;
        }
        Map<List<Body>, Vector3D> possibilitiesPositions = new HashMap<>();
        Map<List<Body>, Double> possibilitiesDistances = new HashMap<>();
        for (List<Body> attachPossibility : closestAttachable.attachPossibilities()) {
            possibilitiesPositions.put(
                    attachPossibility,
                    closestAttachable.attachPossibilitiesPositions(engine.t()).get(attachPossibility));
            possibilitiesDistances.put(
                    attachPossibility,
                    possibilitiesPositions.get(attachPossibility).vectorDistance(basePos).norm());
        }
        List<Body> bestAnchorBlock =
                Collections.min(
                        possibilitiesDistances.keySet(),
                        Comparator.comparingDouble(possibilitiesDistances::get));
        Map<Pair<Body, Body>, Double> bodyDistances = new HashMap<>();
        for (Body b1 : requesterAttachGroup) {
            for (Body b2 : bestAnchorBlock) {
                bodyDistances.put(
                        new Pair<>(b1, b2),
                        b1.position(engine.t()).vectorDistance(b2.position(engine.t())).norm());
            }
        }
        Pair<Body, Body> minDistanceBodies =
                Collections.min(bodyDistances.keySet(), Comparator.comparingDouble(bodyDistances::get));
        Map<Body, Set<Body>> requesterAttachedBodies = requester.attachedBodies();
        Map<Body, Set<Body>> targetAttachedBodies = closestAttachable.attachedBodies();
        int minDistanceIndex1 = requesterAttachGroup.indexOf(minDistanceBodies.first());
        int minDistanceIndex2 = bestAnchorBlock.indexOf(minDistanceBodies.second());
        int nOfAnchors = Math.min(requesterAttachGroup.size(), bestAnchorBlock.size());
        Body requesterBody, targetBody;
        Pair<Body, Body> targetPair;
        for (int i = 0; i < nOfAnchors; ++i) {
            requesterBody =
                    requesterAttachGroup.get((minDistanceIndex1 + i) % requesterAttachGroup.size());
            targetBody =
                    bestAnchorBlock.get(
                            (minDistanceIndex2 + bestAnchorBlock.size() - i) % bestAnchorBlock.size());
            targetPair = new Pair<>(requesterBody, targetBody);
            if (requesterAttachedBodies.get(requesterBody).isEmpty()) {
                if (bodyDistances.get(targetPair) > engine.configuration.maxAttachDistance()) {
                    if (bodyDistances.get(targetPair) > engine.configuration.maxAttractDistance()) {
                        break;
                    }
                    Vector3D force =
                            targetBody.position(engine.t()).vectorDistance(requesterBody.position(engine.t()));
                    force =
                            force.times(
                                    bodyDistances.get(targetPair) / engine.configuration.maxAttractDistance());
                    requesterBody.dBody().addForce(force.x(), force.y(), force.z());
                    targetBody.dBody().addForce(-force.x(), -force.y(), -force.z());
                } else {
                    engine
                            .addSpringJoint(
                                    requesterBody,
                                    targetBody,
                                    engine.configuration.attachSpringConstant(),
                                    engine.configuration.attachDampingConstant())
                            .setDistance(engine.configuration.attachSpringRestDistance());
                    requesterAttachedBodies.get(requesterBody).add(targetBody);
                    targetAttachedBodies.get(targetBody).add(requesterBody);
                }
            }
        }
    }

    @Override
    public void draw(Viewer viewer) {
        // TODO IMPLEMENT
    }
}
