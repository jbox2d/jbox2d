package org.jbox2d.collision;

/**
 * A callback for AABB queries.
 */
public interface ProcessQueryCallback {
	
	/**
	 * This function is called for each overlapping AABB.
	 * @param userData
	 * @return true if the query should continue.
	 */
	public boolean process(Object userData);
}
