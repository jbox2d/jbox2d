/**
 * Created at 7:27:31 AM Jan 21, 2011
 */
package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;

/**
 * @author Daniel Murphy
 */
public class LineJointDef extends JointDef {
	
	/**
	 * The local anchor point relative to body1's origin.
	 */
	public final Vec2 localAnchorA = new Vec2();
	
	/**
	 * The local anchor point relative to body2's origin.
	 */
	public final Vec2 localAnchorB = new Vec2();
	
	/**
	 * The local translation axis in body1.
	 */
	public final Vec2 localAxisA = new Vec2();
	
	/**
	 * Enable/disable the joint limit.
	 */
	public boolean enableLimit;
	
	/**
	 * The lower translation limit, usually in meters.
	 */
	public float lowerTranslation;
	
	/**
	 * The upper translation limit, usually in meters.
	 */
	public float upperTranslation;
	
	/**
	 * Enable/disable the joint motor.
	 */
	public boolean enableMotor;
	
	/**
	 * The maximum motor torque, usually in N-m.
	 */
	public float maxMotorForce;
	
	/**
	 * The desired motor speed in radians per second.
	 */
	public float motorSpeed;
	
	public LineJointDef() {
		type = JointType.LINE;
		localAxisA.set(1, 0);
		enableLimit = false;
		lowerTranslation = 0;
		upperTranslation = 0;
		enableMotor = false;
		maxMotorForce = 0f;
		motorSpeed = 0f;
	}
	
	public void initialize(Body b1, Body b2, Vec2 anchor, Vec2 axis) {
		bodyA = b1;
		bodyB = b2;
		b1.getLocalPointToOut(anchor, localAnchorA);
		b2.getLocalPointToOut(anchor, localAnchorB);
		bodyA.getLocalVectorToOut(axis, localAxisA);
	}
}
