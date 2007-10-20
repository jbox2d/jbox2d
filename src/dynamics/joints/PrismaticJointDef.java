package dynamics.joints;

import common.Vec2;

public class PrismaticJointDef extends JointDef {
    public Vec2 anchorPoint;

    public Vec2 axis;

    public float lowerTranslation;

    public float upperTranslation;

    public float motorForce;

    public float motorSpeed;

    public boolean enableLimit;

    public boolean enableMotor;

    public PrismaticJointDef() {
        type = JointType.PRISMATIC_JOINT;
        anchorPoint = new Vec2(0.0f, 0.0f);
        axis = new Vec2(1.0f, 0.0f);
        lowerTranslation = 0.0f;
        upperTranslation = 0.0f;
        motorForce = 0.0f;
        motorSpeed = 0.0f;
        enableLimit = false;
        enableMotor = false;
    }
}