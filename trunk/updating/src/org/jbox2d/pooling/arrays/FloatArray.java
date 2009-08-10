package org.jbox2d.pooling.arrays;

public class FloatArray extends DynamicTLArray<Float> {
	@Override
	protected Float[] getInitializedArray(int argLength) {
		return new Float[argLength];
	}
}
