package org.jbox2d.structs.collision.broadphase;

import org.jbox2d.collision.broadphase.DynamicTreeNode;

public interface QueryCallback {
	public void queryCallback(DynamicTreeNode node);
}
