package org.jbox2d.dynamics;

import org.jbox2d.common.Sweep;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;

/**
 * A rigid body. These are created via World::createBody.
 *
 * @author daniel
 */

public class Body {
	public static final int e_islandFlag = 0x0001;
	public static final int e_sleepFlag = 0x0002;
	public static final int e_allowSleepFlag = 0x0004;
	public static final int e_bulletFlag = 0x0008;
	public static final int e_fixedRotationFlag = 0x0010;
	
	public static final int TYPE_STATIC = 0;
	public static final int TYPE_DYNAMIC = 1;
	public static final int MAX_TYPES = 2;
	
	
	public int m_flags;
	public int m_type;
	
	public int m_islandIndex;
	
	/**
	 * The body orgin transform.
	 */
	public final Transform m_xf = new Transform();
	
	/**
	 * The swept motion for CCD
	 */
	public final Sweep m_sweep = new Sweep();
	
	public final Vec2 m_linearVelocity = new Vec2();
	
	public float m_anglularVelocity;
	
	public final Vec2 m_force = new Vec2();
	
	public World m_world;
	
	public Body m_prev;
	public Body m_next;
	
	public Fixture m_fixtureList;
	public int m_fixtureCount;
	
	public Joint m_jointList;
	public ContactEdge m_contactList;
	
	public float m_mass, m_invMass;
	public float m_I, m_invI;
	
	public float m_linearDampening;
	public float m_anglularDampening;
	
	public float m_sleeptime;
	
	public Object m_userData;
	
}
