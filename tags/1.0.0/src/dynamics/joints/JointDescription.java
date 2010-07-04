package dynamics.joints;

import dynamics.Body;

public class JointDescription {
    public JointType type;

    public Body body1;

    public Body body2;

    public JointDescription() {
        type = JointType.unknownJoint;
        body1 = null;
        body2 = null;
    }

}
