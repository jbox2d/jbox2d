package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.structs.dynamics.joints.JointEdge;
import org.jbox2d.structs.dynamics.joints.JointType;

// updated to rev 100
/**
 * The base joint class. Joints are used to raint two bodies together in
 * various fashions. Some joints also feature limits and motors.
 * @author Daniel Murphy
 */
public abstract class Joint {
	
	public static Joint create(JointDef def){
		Joint joint = null;
		if(def instanceof MouseJointDef){
			return new MouseJoint((MouseJointDef) def);
		}
		return joint;
	}
	
	public static void destroy(Joint joint){
		// nothing yet
	}
	
	
	public JointType m_type;
	public Joint m_prev;
	public Joint m_next;
	public JointEdge m_edgeA;
	public JointEdge m_edgeB;
	public Body m_bodyA;
	public Body m_bodyB;
	
	public boolean m_islandFlag;
	public boolean m_collideConnected;
	
	public Object m_userData;
	
	// Cache here per time step to reduce cache misses.
	Vec2 m_localCenterA, m_localCenterB;
	float m_invMassA, m_invIA;
	float m_invMassB, m_invIB;
	
	protected Joint(JointDef def){
		assert(def.bodyA != def.bodyB);
		
		m_type = def.type;
		m_prev = null;
		m_next = null;
		m_bodyA = def.bodyA;
		m_bodyB = def.bodyB;
		m_collideConnected = def.collideConnected;
		m_islandFlag = false;
		m_userData = def.userData;
		
		m_edgeA = new JointEdge();
		m_edgeA.joint = null;
		m_edgeA.other = null;
		m_edgeA.prev = null;
		m_edgeA.next = null;
		
		m_edgeB = new JointEdge();
		m_edgeB.joint = null;
		m_edgeB.other = null;
		m_edgeB.prev = null;
		m_edgeB.next = null;
	}
	
	/**
	 * get the type of the concrete joint.
	 * @return
	 */
	public JointType getType(){
		return m_type;
	}

	/**
	 * get the first body attached to this joint.
	 */
	public Body getBodyA(){
		return m_bodyA;
	}

	/**
	 * get the second body attached to this joint.
	 * @return
	 */
	public Body getBodyB(){
		return m_bodyB;
	}

	/**
	 * get the anchor point on bodyA in world coordinates.
	 * @return
	 */
	public abstract Vec2 getAnchorA();

	/**
	 * get the anchor point on bodyB in world coordinates.
	 * @return
	 */
	public abstract Vec2 getAnchorB();

	/**
	 * get the reaction force on body2 at the joint anchor in Newtons.
	 * @param inv_dt
	 * @return
	 */
	public abstract Vec2 getReactionForce(float inv_dt);

	/**
	 * get the reaction torque on body2 in N*m.
	 * @param inv_dt
	 * @return
	 */
	public abstract float getReactionTorque(float inv_dt);

	/**
	 * get the next joint the world joint list.
	 */
	public Joint getNext(){
		return m_next;
	}

	/**
	 * get the user data pointer.
	 */
	public Object getUserData(){
		return m_userData;
	}

	/**
	 * Set the user data pointer.
	 */
	public void setUserData(Object data){
		m_userData = data;
	}

	/**
	 * Short-cut function to determine if either body is inactive.
	 * @return
	 */
	public boolean IsActive(){
		return m_bodyA.isActive() && m_bodyB.isActive();
	}
	
	public abstract void initVelocityConstraints(TimeStep step);
	public abstract void solveVelocityConstraints(TimeStep step);
	
	/**
	 * This returns true if the position errors are within tolerance.
	 * @param baumgarte
	 * @return
	 */
	public abstract boolean solvePositionConstraints(float baumgarte);
}
