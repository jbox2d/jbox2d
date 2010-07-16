package org.jbox2d.pooling.stacks;

import org.jbox2d.dynamics.Island;

public class IslandStack extends DynamicTLStack<Island> {
	@Override
	protected Island newObjectInstance() {
		return new Island();
	}
}
