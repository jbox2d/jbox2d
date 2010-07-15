package org.jbox2d.callbacks;

import org.jbox2d.collision.broadphase.DynamicTreeNode;

public interface TreeCallback {
	
	/**
	 * Callback from a query request.  
	 * @param node
	 * @return if the query should be continued
	 */
	public boolean treeCallback(DynamicTreeNode node);
}
