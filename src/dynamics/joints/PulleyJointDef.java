package dynamics.joints;

import common.Vec2;

public class PulleyJointDef extends JointDef {

    public Vec2 groundPoint1;

    public Vec2 groundPoint2;

    public Vec2 anchorPoint1;

    public Vec2 anchorPoint2;

    public float maxLength1;

    public float maxLength2;

    public float ratio;

    public PulleyJointDef() {
        type = JointType.PULLEY_JOINT;
        groundPoint1 = new Vec2(-1.0f, 1.0f);
        groundPoint2 = new Vec2(1.0f, 1.0f);
        anchorPoint1 = new Vec2(-1.0f, 0.0f);
        anchorPoint2 = new Vec2(1.0f, 0.0f);
        maxLength1 = 0.5f * PulleyJoint.MIN_PULLEY_LENGTH;
        maxLength2 = 0.5f * PulleyJoint.MIN_PULLEY_LENGTH;
        ratio = 1.0f;
        collideConnected = true;
    }
}
