package actions;

import ad.Attachable;
import bodies.Body;
import engine.Ode4jEngine;

import java.util.HashSet;
import java.util.List;

public class RequestDetachment implements Action {
    private final Attachable requester;
    private final List<Body> requesterAttachGroup;

    public RequestDetachment(Attachable requester, List<Body> requesterAttachGroup) {
        this.requester = requester;
        this.requesterAttachGroup = requesterAttachGroup;
    }

    @Override
    public void execute(Ode4jEngine engine) {
        //TODO TEST
        for (Body requesterBody : requesterAttachGroup) {
            for (Body attachedBody : requester.attachedBodies().get(requesterBody)) {
                engine.removeSpringJoints(requesterBody, attachedBody);
            }
            requester.attachedBodies().put(requesterBody, new HashSet<>());
        }
    }
}
