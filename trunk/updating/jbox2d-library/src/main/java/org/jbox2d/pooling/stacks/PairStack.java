package org.jbox2d.pooling.stacks;

import org.jbox2d.collision.broadphase.Pair;

public class PairStack extends DynamicTLStack<Pair> {

	@Override
	protected Pair newObjectInstance() {
		return new Pair();
	}

}
