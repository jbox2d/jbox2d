/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;

//Updated to rev. 56->108 of b2RevoluteJoint.cpp/.h

/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
public class RevoluteJointDef extends JointDef {
	public RevoluteJointDef() {
		type = JointType.REVOLUTE_JOINT;
		localAnchor1 = new Vec2(0.0f, 0.0f);
		localAnchor2 = new Vec2(0.0f, 0.0f);
		referenceAngle = 0.0f;
		lowerAngle = 0.0f;
		upperAngle = 0.0f;
		maxMotorTorque = 0.0f;
		motorSpeed = 0.0f;
		enableLimit = false;
		enableMotor = false;
	}

	/// Initialize the bodies, anchors, and reference angle using the world
	/// anchor.
	public void initialize(Body b1, Body b2, Vec2 anchor) {
		body1 = b1;
		body2 = b2;
		localAnchor1 = body1.getLocalPoint(anchor);
		localAnchor2 = body2.getLocalPoint(anchor);
		referenceAngle = body2.getAngle() - body1.getAngle();
	}

	/// The local anchor point relative to body1's origin.
	public Vec2 localAnchor1;

	/// The local anchor point relative to body2's origin.
	public Vec2 localAnchor2;

	/// The body2 angle minus body1 angle in the reference state (radians).
	public float referenceAngle;

	/// A flag to enable joint limits.
	public boolean enableLimit;

	/// The lower angle for the joint limit (radians).
	public float lowerAngle;

	/// The upper angle for the joint limit (radians).
	public float upperAngle;

	/// A flag to enable the joint motor.
	public boolean enableMotor;

	/// The desired motor speed. Usually in radians per second.
	public float motorSpeed;

	/// The maximum motor torque used to achieve the desired motor speed.
	/// Usually in N-m.
	public float maxMotorTorque;
}
