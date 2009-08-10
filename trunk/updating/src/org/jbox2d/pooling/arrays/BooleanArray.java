package org.jbox2d.pooling.arrays;

public class BooleanArray extends DynamicTLArray<Boolean> {

	@Override
	protected Boolean[] getInitializedArray(int argLength) {
		Boolean[] ray = new Boolean[argLength];
		for(int i=0; i<argLength; i++){
			ray[i] = false;
		}
		return ray;
	}

}
