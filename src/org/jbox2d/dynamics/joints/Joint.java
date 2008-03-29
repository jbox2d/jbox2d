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
import org.jbox2d.dynamics.TimeStep;


//Updated to rev 56->97 of b2Joint.cpp/.h

public abstract class Joint {

    public JointType m_type;

    public Joint m_prev;

    public Joint m_next;

    public JointEdge m_node1;

    public JointEdge m_node2;

    public Body m_body1;

    public Body m_body2;

    public boolean m_islandFlag;

    public boolean m_collideConnected;

    public Object m_userData;

    public Joint(JointDef description) {
        m_type = description.type;
        m_prev = null;
        m_next = null;
        m_node1 = new JointEdge();
        m_node2 = new JointEdge();
        m_body1 = description.body1;
        m_body2 = description.body2;
        m_collideConnected = description.collideConnected;
        m_islandFlag = false;
        m_userData = description.userData;
    }

    // ewjordan: I've added a Destroy method because although
    // these usually just deallocate memory, it is possible that
    // Erin may alter them to do more nontrivial things, and we
    // should be prepared for this possibility.
    public static void destroy(Joint j) {
        j.destructor();
        return;
    }

    public void destructor() {
    }

    public static Joint create(JointDef description) {
        Joint joint = null;

        if (description.type == JointType.DISTANCE_JOINT) {
            joint = new DistanceJoint((DistanceJointDef) description);
        }
        else if (description.type == JointType.MOUSE_JOINT) {
            joint = new MouseJoint((MouseJointDef) description);
        }
        else if (description.type == JointType.PRISMATIC_JOINT) {
            joint = new PrismaticJoint((PrismaticJointDef) description);
        }
        else if (description.type == JointType.REVOLUTE_JOINT) {
            joint = new RevoluteJoint((RevoluteJointDef) description);
        }
        else if (description.type == JointType.PULLEY_JOINT) {
            joint = new PulleyJoint((PulleyJointDef) description);
        }
        else if (description.type == JointType.GEAR_JOINT) {
            joint = new GearJoint((GearJointDef) description);
        }
        else {
            assert false;
        }

        return joint;
    }

    /** Get the type of the concrete joint. */
    public JointType getType() {
    	return m_type;
    }
    
	/** Get the first body attached to this joint. */
    public Body getBody1() {
        return m_body1;
    }
    
	/** Get the second body attached to this joint. */
    public Body getBody2() {
        return m_body2;
    }

    
    /** Get the anchor point on body1 in world coordinates. */
	public abstract Vec2 getAnchor1();

	/** Get the anchor point on body2 in world coordinates. */
	public abstract Vec2 getAnchor2();
	
    /** Get the reaction force on body2 at the joint anchor. */
	public abstract Vec2 getReactionForce();

	/** Get the reaction torque on body2. */
	public abstract float getReactionTorque();
    
    /** Get the next joint the world joint list. */
    public Joint getNext() {
        return m_next;
    }
    
    /** Get the user data pointer. */
    public Object getUserData() {
        return m_userData;
    }

    public abstract void initVelocityConstraints(TimeStep step);

    public abstract void solveVelocityConstraints(TimeStep step);

    public void initPositionConstraints() {
        return;
    }
    
    /** This returns true if the position errors are within tolerance. */
    public abstract boolean solvePositionConstraints();

}
