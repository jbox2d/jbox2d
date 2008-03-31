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

import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;

// Updated to rev 130 of b2PulleyJoint.cpp/.h

public class PulleyJointDef extends JointDef {

	public PulleyJointDef() {
		type = JointType.PULLEY_JOINT;
		groundAnchor1 = new Vec2(-1.0f, 1.0f);
		groundAnchor2 = new Vec2(1.0f, 1.0f);
		localAnchor1 = new Vec2(-1.0f, 0.0f);
		localAnchor2 = new Vec2(1.0f, 0.0f);
		length1 = 0.0f;
		maxLength1 = 0.0f;
		length2 = 0.0f;
		maxLength2 = 0.0f;
		ratio = 1.0f;
		collideConnected = true;
	}

	/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
	public void initialize(Body b1, Body b2,
					Vec2 ga1, Vec2 ga2,
					Vec2 anchor1, Vec2 anchor2,
					float r){
		body1 = b1;
		body2 = b2;
		groundAnchor1 = ga1;
		groundAnchor2 = ga2;
		localAnchor1 = body1.getLocalPoint(anchor1);
		localAnchor2 = body2.getLocalPoint(anchor2);
		Vec2 d1 = anchor1.sub(ga1);
		length1 = d1.length();
		Vec2 d2 = anchor2.sub(ga2);
		length2 = d2.length();
		ratio = r;
		assert(ratio > Settings.EPSILON);
		float C = length1 + ratio * length2;
		maxLength1 = C - ratio * PulleyJoint.MIN_PULLEY_LENGTH;
		maxLength2 = (C - PulleyJoint.MIN_PULLEY_LENGTH) / ratio;
	}
	
	/// The first ground anchor in world coordinates. This point never moves.
	public Vec2 groundAnchor1;

	/// The second ground anchor in world coordinates. This point never moves.
	public Vec2 groundAnchor2;

	/// The local anchor point relative to body1's origin.
	public Vec2 localAnchor1;

	/// The local anchor point relative to body2's origin.
	public Vec2 localAnchor2;

	/// The a reference length for the segment attached to body1.
	public float length1;

	/// The maximum length of the segment attached to body1.
	public float maxLength1;

	/// The a reference length for the segment attached to body2.
	public float length2;

	/// The maximum length of the segment attached to body2.
	public float maxLength2;

	/// The pulley ratio, used to simulate a block-and-tackle.
	public float ratio;
}
