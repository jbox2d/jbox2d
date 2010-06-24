package org.jbox2d.structs.collision;

// updated to rev 100
/**
 * This is used for determining the state of contact points.
 * @author Daniel Murphy
 */
public enum PointState {
	/**
	 * point does not exist
	 */
	NULL_STATE,
	/**
	 * point was added in the update
	 */
	ADD_STATE,
	/**
	 * point persisted across the update
	 */
	PERSIST_STATE,
	/**
	 * point was removed in the update
	 */
	REMOVE_STATE
}
