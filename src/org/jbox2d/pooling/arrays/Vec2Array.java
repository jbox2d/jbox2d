package org.jbox2d.pooling.arrays;

import org.jbox2d.common.Vec2;

public class Vec2Array extends DynamicTLArray<Vec2> {

	@Override
	protected Vec2[] getInitializedArray(int argLength) {
		Vec2[] ray = new Vec2[argLength];
		for(int i=0; i<ray.length; i++){
			ray[i] = new Vec2();
		}
		return ray;
	}

}
