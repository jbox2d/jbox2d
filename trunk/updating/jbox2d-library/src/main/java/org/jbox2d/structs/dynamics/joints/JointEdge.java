package org.jbox2d.structs.dynamics.joints;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.joints.Joint;

//updated to rev 100 - ec
/**
 * A joint edge is used to connect bodies and joints together
 * in a joint graph where each body is a node and each joint
 * is an edge. A joint edge belongs to a doubly linked list
 * maintained in each attached body. Each joint has two joint
 * nodes, one for each attached body.
 * @author Daniel
 */
public class JointEdge {
	
	/**
	 * Provides quick access to the other body attached
	 */
	public Body other = null;
	
	/**
	 * the joint
	 */
	public Joint joint = null;
	
	/**
	 * the previous joint edge in the body's joint list
	 */
	public JointEdge prev = null;
	
	/**
	 * the next joint edge in the body's joint list
	 */
	public JointEdge next = null;
}
