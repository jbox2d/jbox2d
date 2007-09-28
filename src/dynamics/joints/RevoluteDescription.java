package dynamics.joints;

import common.Vec2;

public class RevoluteDescription extends JointDescription {

	Vec2 anchorPoint;
	float lowerAngle;
	float upperAngle;
	float motorTorque;
	float motorSpeed;
	boolean enableLimit;
	boolean enableMotor;

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
