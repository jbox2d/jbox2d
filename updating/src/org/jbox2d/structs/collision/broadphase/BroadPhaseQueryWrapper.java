package org.jbox2d.structs.collision.broadphase;

import org.jbox2d.collision.DynamicTree;
import org.jbox2d.collision.DynamicTreeNode;
import org.jbox2d.structs.collision.tree.TreeQueryCallback;

/**
 * This wrapper converts tree proxy user data to broad-phase proxy user data.
 *
 * @author daniel
 */
public class BroadPhaseQueryWrapper implements TreeQueryCallback {
	
	public QueryCallback callback;
	public DynamicTree tree;
	
	public void queryCallback(DynamicTreeNode argNode){
		callback.queryCallback(argNode.userData);
	}
}
