package org.jbox2d.pooling.arrays;

public class TLIntegerArray extends DynamicTLArray<Integer> {
	@Override
	protected final Integer[] getInitializedArray(int argLength) {
		return new Integer[argLength];
	}
}
