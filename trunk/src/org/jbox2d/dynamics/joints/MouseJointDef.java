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

//Updated to rev 130 of b2MouseJoint.cpp/.h

public class MouseJointDef extends JointDef {
	/**
	 * The initial world target point. This is assumed
	 * to coincide with the body anchor initially.
	 */
	public Vec2 target;

    /** 
     * The maximum constraint force that can be exerted
	 * to move the candidate body. Usually you will express
	 * as some multiple of the weight (multiplier * mass * gravity).
	 */
    public float maxForce;

    /** The response speed. */
    public float frequencyHz;
  
    /** The damping ratio. 0 = no damping, 1 = critical damping. */
	public float dampingRatio;
    
    /** The time step used in the simulation. */
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
