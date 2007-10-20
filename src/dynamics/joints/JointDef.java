package dynamics.joints;

import dynamics.Body;

public class JointDef {
    public JointType type;

    public Body body1;

    public Body body2;

    public Object userData;

    public boolean collideConnected;

    public JointDef() {
        type = JointType.UNKNOWN_JOINT;
        body1 = null;
        body2 = null;
        userData = null;
        collideConnected = false;
    }
}
