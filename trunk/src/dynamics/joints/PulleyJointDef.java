package dynamics.joints;

import common.Vec2;

public class PulleyJointDef extends JointDef {

    public PulleyJointDef() {
        type = JointType.pulleyJoint;
        groundPoint1 = new Vec2(-1.0f, 1.0f);
        groundPoint2 = new Vec2(1.0f, 1.0f);
        anchorPoint1 = new Vec2(-1.0f, 0.0f);
        anchorPoint2 = new Vec2(1.0f, 0.0f);
        maxLength1 = 0.5f * PulleyJoint.minPulleyLength;
        maxLength2 = 0.5f * PulleyJoint.minPulleyLength;
        ratio = 1.0f;
        collideConnected = true;
    }

    public Vec2 groundPoint1;
    public Vec2 groundPoint2;
    public Vec2 anchorPoint1;
    public Vec2 anchorPoint2;
    public float maxLength1;
    public float maxLength2;
    public float ratio;
}
