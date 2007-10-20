package dynamics;

import dynamics.joints.Joint;

public class WorldListener {
    public void Destructor() {
        
    }
    
    public void NotifyJointDestroyed(Joint joint) {
        
    }
    
    public BoundaryResponse NotifyBoundaryViolated(Body body) {
        //NOT_USED(body);
        return BoundaryResponse.freezeBody;
    }
}