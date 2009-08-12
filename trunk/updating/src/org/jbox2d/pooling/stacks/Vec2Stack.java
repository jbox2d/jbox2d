package org.jbox2d.pooling.stacks;

import org.jbox2d.common.Vec2;

public class Vec2Stack extends DynamicTLStack<Vec2> {

	@Override
	protected Vec2 newObjectInstance() {
		return new Vec2();
	}

}
