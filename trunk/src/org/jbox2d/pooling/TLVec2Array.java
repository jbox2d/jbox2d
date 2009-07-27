package org.jbox2d.pooling;

import org.jbox2d.common.Vec2;

public class TLVec2Array extends ThreadLocal<Vec2[]> {
	private final int length;
	public TLVec2Array(int argLength){
		length = argLength;
	}
	
	protected Vec2[] initialValue(){
		assert(length > 0);
		Vec2[] ray = new Vec2[length];
		for(int i=0; i < length; i++){
			ray[i] = new Vec2();
		}
		return ray;
	}
}
