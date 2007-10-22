package dynamics.joints;

import common.Vec2;

public class MouseJointDef extends JointDef {
    public Vec2 target;

    public float maxForce;

    public float frequencyHz;

    public float dampingRatio;

    public float timeStep;

    public MouseJointDef() {
        type = JointType.MOUSE_JOINT;
        target = new Vec2(0.0f, 0.0f);
        maxForce = 0.0f;
        frequencyHz = 5.0f;
        dampingRatio = 0.7f;
        timeStep = 1.0f / 60.0f;
    }
}
