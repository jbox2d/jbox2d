package dynamics.joints;

import common.Vec2;

public class RevoluteDescription extends JointDescription {

    public Vec2 anchorPoint;

    public float lowerAngle;

    public float upperAngle;

    public float motorTorque;

    public float motorSpeed;

    public boolean enableLimit;

    public boolean enableMotor;

    public RevoluteDescription() {
        type = JointType.revoluteJoint;
        anchorPoint = new Vec2(0.0f, 0.0f);
        lowerAngle = 0.0f;
        upperAngle = 0.0f;
        motorTorque = 0.0f;
        motorSpeed = 0.0f;
        enableLimit = false;
        enableMotor = false;
    }
}
