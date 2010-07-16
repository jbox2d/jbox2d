package org.jbox2d.callbacks;

import org.jbox2d.collision.broadphase.DynamicTree;
import org.jbox2d.collision.broadphase.DynamicTreeNode;
import org.jbox2d.structs.collision.RayCastInput;

// updated to rev 100

/**
 * callback for {@link DynamicTree}
 * @author Daniel Murphy
 *
 */
public interface TreeRayCastCallback {
	/**
	 * 
	 * @param argInput
	 * @param argNode
	 * @return the fraction to the node
	 */
	public float raycastCallback( RayCastInput argInput, DynamicTreeNode argNode);
}
