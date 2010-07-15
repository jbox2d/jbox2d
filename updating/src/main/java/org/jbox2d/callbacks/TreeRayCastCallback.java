package org.jbox2d.callbacks;

import org.jbox2d.collision.broadphase.DynamicTreeNode;
import org.jbox2d.structs.collision.RayCastInput;

public interface TreeRayCastCallback {
	/**
	 * 
	 * @param argInput
	 * @param argNode
	 * @return the fraction to the node
	 */
	public float raycastCallback( RayCastInput argInput, DynamicTreeNode argNode);
}
