package org.jbox2d.pooling.arrays;

public class IntegerArray extends DynamicTLArray<Integer> {
	@Override
	protected final Integer[] getInitializedArray(int argLength) {
		return new Integer[argLength];
	}
}
