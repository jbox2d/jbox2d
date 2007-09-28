package dynamics.joints;

import common.Vec2;

public class PrismaticJointDescription extends JointDescription {
	Vec2 anchorPoint;
	Vec2 axis;
	float lowerTranslation;
	float upperTranslation;
	float motorForce;
	float motorSpeed;
	boolean enableLimit;
	boolean enableMotor;

	PrismaticJointDescription() {
		type = JointType.prismaticJoint;
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