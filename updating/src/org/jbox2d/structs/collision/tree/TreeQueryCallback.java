package org.jbox2d.structs.collision.tree;

import org.jbox2d.collision.DynamicTreeNode;

public interface TreeQueryCallback {
	public void queryCallback(DynamicTreeNode node);
}
