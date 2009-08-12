package org.jbox2d.pooling.stacks;

import org.jbox2d.structs.collision.tree.DynamicTreeNode;

public class DynamicTreeNodeStack extends DynamicTLStack<DynamicTreeNode> {

	@Override
	protected DynamicTreeNode newObjectInstance() {
		return new DynamicTreeNode();
	}
	
}
