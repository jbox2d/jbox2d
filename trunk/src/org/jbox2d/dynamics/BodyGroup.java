/**
 * 
 */
package org.jbox2d.dynamics;

import java.util.HashSet;
import java.util.Set;

/**
 * Holder for a group of bodies.
 * @author eric
 */
public class BodyGroup {
	private final Set<BodyGroup> children = new HashSet<BodyGroup>();
	private final Set<Body> bodies = new HashSet<Body>();
	private final World world;
	
	/** Create a BodyGroup from a group of bodies. */
	public BodyGroup(Body... bodies) {
		World w = null;
		for (Body b:bodies) {
			this.bodies.add(b);
			if (w == null) w = b.getWorld();
			else if (b.getWorld() != w) assert(false):"Cannot add bodies from different worlds to a BodyGroup";
		}
		world = w;
	}
	
	/** Create a BodyGroup from a group of BodyGroups. */
	public BodyGroup(BodyGroup... groups) {
		World w = null;
		for (BodyGroup bg:groups) {
			this.children.add(bg);
			if (w == null) w = bg.getWorld();
			else if (bg.getWorld() != w) assert(false):"Cannot add BodyGroups from different worlds to a BodyGroup";
		}
		world = w;
	}
	
	/** Get child BodyGroups. Does not return children-of-children.*/
	public Set<BodyGroup> getChildren() {
		return children;
	}
	
	/** 
	 * Get bodies. Does not return bodies that are members of child BodyGroups.
	 * @see #getBodiesDeep() 
	 */
	public Set<Body> getBodies() {
		return bodies;
	}
	
	/** 
	 * Get all bodies in this group and all children, recursively.
	 * @see #getBodies() for non-recursive version
	 */
	public Set<Body> getBodiesDeep() {
		Set<Body> res = new HashSet<Body>();
		res.addAll(bodies);
		for (BodyGroup bg:children) {
			res.addAll(bg.getBodiesDeep());
		}
		return res;
	}
	
	/** @return the world this BodyGroup lives in */
	public World getWorld() {
		return world;
	}
	
	/**
	 * Create a BodyGroup by traversing the joint tree from a seed body.
	 * @param seed
	 */
	public static BodyGroup fromSeedConnectedByJoints(Body seed) {
		Set<Body> bodies = seed.getConnectedDynamicBodyIsland();
		return new BodyGroup(bodies.toArray(new Body[bodies.size()]));
	}
}
