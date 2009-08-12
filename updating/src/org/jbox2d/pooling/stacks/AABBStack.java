package org.jbox2d.pooling.stacks;

import org.jbox2d.collision.AABB;

public class AABBStack extends DynamicTLStack<AABB> {
	@Override
	protected AABB newObjectInstance() {
		return new AABB();
	}

}
