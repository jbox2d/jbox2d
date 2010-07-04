package org.jbox2d.structs.collision;

import org.jbox2d.collision.broadphase.DynamicTreeNode;

public interface RayCastCallback {
	/**
	 * 
	 * @param argInput
	 * @param argNode
	 * @return the fraction to the node
	 */
	public float raycastCallback( RayCastInput argInput, DynamicTreeNode argNode);
}
