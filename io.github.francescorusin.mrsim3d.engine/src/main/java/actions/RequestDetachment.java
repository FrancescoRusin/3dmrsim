package actions;

import ad.Attachable;
import bodies.Body;
import engine.Ode4jEngine;
import viewer.Viewer;

import java.util.*;

public class RequestDetachment implements Action {
    private final Attachable requester;
    private final List<Body> requesterAttachGroup;

    public RequestDetachment(Attachable requester, List<Body> requesterAttachGroup) {
        this.requester = requester;
        this.requesterAttachGroup = requesterAttachGroup;
    }

    @Override
    public void execute(Ode4jEngine engine) {
        Map<Body, Set<Body>> requesterAttachedBodies = requester.attachedBodies();
        for (Body requesterBody : requesterAttachGroup) {
            for (Body attachedBody : requesterAttachedBodies.get(requesterBody)) {
                engine.removeSpringJoints(requesterBody, attachedBody);
                if (attachedBody instanceof Attachable attachableBody) {
                    attachableBody.attachedBodies().get(attachedBody).remove(requesterBody);
                } else if (engine.componentToAgentMapper.get(attachedBody) instanceof Attachable attachableAgent) {
                    attachableAgent.attachedBodies().get(attachedBody).remove(requesterBody);
                }
            }
            requesterAttachedBodies.get(requesterBody).clear();
        }
    }

    @Override
    public void draw(Viewer viewer) {
        //TODO IMPLEMENT
    }
}