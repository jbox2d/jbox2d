package dynamics.joints;

import common.Vec2;

public class MouseDef extends JointDef {
    public Vec2 target;

    public float beta;

    public float motorForce;

    public float length;

    public MouseDef() {
        type = JointType.MOUSE_JOINT;
        target = new Vec2(0.0f, 0.0f);
        motorForce = 0.0f;
        beta = 0.2f;
        length = 1.0f;
    }
}
