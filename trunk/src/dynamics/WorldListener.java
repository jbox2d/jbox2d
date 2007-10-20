package dynamics;

import dynamics.joints.Joint;

public class WorldListener {
    public void destructor() {
    }

    public void notifyJointDestroyed(Joint joint) {
    }

    public BoundaryResponse notifyBoundaryViolated(Body body) {
        // NOT_USED(body);
        return BoundaryResponse.FREEZE_BODY;
    }
}