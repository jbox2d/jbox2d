package dynamics.joints;

import common.Vec2;

public class MouseDescription extends JointDescription {
    public Vec2 target;

    public float beta;

    public float motorForce;

    public float length;

    public MouseDescription() {
        type = JointType.mouseJoint;
        target = new Vec2(0.0f, 0.0f);
        motorForce = 0.0f;
        beta = 0.2f;
        length = 1.0f;
    }

}
