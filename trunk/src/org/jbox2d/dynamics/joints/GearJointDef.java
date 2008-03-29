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

//Updated to rev. 97 of b2GearJoint.cpp/.h


/**
 * 
 * Gear joint definition. This definition requires two existing
 * revolute or prismatic joints (any combination will work).
 * The provided joints must attach a dynamic body to a static body.<BR><BR>
 * A gear joint is used to connect two joints together. Either joint
 * can be a revolute or prismatic joint. You specify a gear ratio
 * to bind the motions together:
 * coordinate1 + ratio * coordinate2 = constant
 * The ratio can be negative or positive. If one joint is a revolute joint
 * and the other joint is a prismatic joint, then the ratio will have units
 * of length or units of 1/length.
 * <BR><em>Warning</em>: The revolute and prismatic joints must be attached to
 * fixed bodies (which must be body1 on those joints).
 */
public class GearJointDef extends JointDef {
	/** The first revolute/prismatic joint attached to the gear joint. */
	public Joint joint1;

	/** The second revolute/prismatic joint attached to the gear joint. */
	public Joint joint2;

	/**
	 * The gear ratio.
	 * @see org.jbox2d.dynamics.joints.GearJoint for explanation.
	 */
	public float ratio;
	
	public GearJointDef() {
		type = JointType.GEAR_JOINT;
		joint1 = null;
		joint2 = null;
		ratio = 1.0f;
	}
	
}
