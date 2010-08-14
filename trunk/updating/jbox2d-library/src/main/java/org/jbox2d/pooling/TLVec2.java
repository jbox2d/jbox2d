package org.jbox2d.pooling;

import org.jbox2d.common.Vec2;

public class TLVec2 extends CustThreadLocal<Vec2> {
	protected Vec2 initialValue(){
		return new Vec2();
	}
}
