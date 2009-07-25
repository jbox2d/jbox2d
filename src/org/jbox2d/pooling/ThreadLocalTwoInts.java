package org.jbox2d.pooling;

public class ThreadLocalTwoInts extends ThreadLocal<Integer[]> {
	protected Integer[] initialValue(){
		Integer ints[] = {
		   0,0
		};
		return ints;
	}
}
