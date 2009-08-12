package org.jbox2d.structs.collision;

import org.jbox2d.collision.DynamicTreeNode;

public interface RayCastCallback {
	public void raycastCallback( RayCastOutput argOutput, RayCastInput argInput, DynamicTreeNode argNode);
}
