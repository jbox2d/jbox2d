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

//Updated to rev 56->130 of b2DistanceJoint.cpp/.h

/**
 * Definition for a distance joint.  A distance joint
 * keeps two points on two bodies at a constant distance
 * from each other.
 */
public class DistanceJointDef extends JointDef {
	/** The local anchor point relative to body1's origin. */
	public Vec2 localAnchor1;
	
	/** The local anchor point relative to body2's origin. */
	public Vec2 localAnchor2;
	
	/** The equilibrium length between the anchor points. */
	public float length;
    
	public DistanceJointDef() {
		type = JointType.DISTANCE_JOINT;
		localAnchor1 = new Vec2(0.0f, 0.0f);
		localAnchor2 = new Vec2(0.0f, 0.0f);
		length = 1.0f;
	}
	
	/**
	 * Initialize the bodies, anchors, and length using the world
	 * anchors.
	 */
    public void initialize(Body b1, Body b2, Vec2 anchor1, Vec2 anchor2) {	
    	body1 = b1;
    	body2 = b2;
    	localAnchor1 = body1.getLocalPoint(anchor1);
    	localAnchor2 = body2.getLocalPoint(anchor2);
    	Vec2 d = anchor2.sub(anchor1);
		length = d.length();
	}
}
