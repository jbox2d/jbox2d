package org.jbox2d.callbacks;

import org.jbox2d.collision.broadphase.DynamicTree;
import org.jbox2d.collision.broadphase.DynamicTreeNode;

// update to rev 100
/**
 * callback for {@link DynamicTree}
 * @author Daniel Murphy
 *
 */
public interface TreeCallback {
	
	/**
	 * Callback from a query request.  
	 * @param node
	 * @return if the query should be continued
	 */
	public boolean treeCallback(DynamicTreeNode node);
}
