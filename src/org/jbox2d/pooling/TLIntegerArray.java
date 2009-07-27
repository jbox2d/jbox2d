package org.jbox2d.pooling;

public class TLIntegerArray extends ThreadLocal<Integer[]> {
	private final int length;
	
	public TLIntegerArray(int argLength){
		length = argLength;
	}
	
	protected Integer[] initialValue(){
		assert(length > 0);
		Integer ints[] = new Integer[length];
		for(int i=0; i<length; i++){
			ints[i] = new Integer(0);
		}
		return ints;
	}
}
