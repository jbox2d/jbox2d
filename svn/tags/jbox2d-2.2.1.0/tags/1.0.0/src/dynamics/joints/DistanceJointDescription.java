package dynamics.joints;

import common.Vec2;

public class DistanceJointDescription extends JointDescription {
    public Vec2 anchorPoint1;

    public Vec2 anchorPoint2;

    public DistanceJointDescription() {
        type = JointType.distanceJoint;
        anchorPoint1 = new Vec2(0.0f, 0.0f);
        anchorPoint2 = new Vec2(0.0f, 0.0f);
    }
}
