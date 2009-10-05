package org.jbox2d.structs.collision.broadphase;

import org.jbox2d.collision.broadphase.DynamicTreeNode;

public interface QueryCallback {
	
	/**
	 * Callback from a query request.  
	 * @param node
	 * @return if the query should be continued
	 */
	public boolean queryCallback(DynamicTreeNode node);
}
