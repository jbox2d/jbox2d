package org.jbox2d.pooling;

import org.jbox2d.common.Vec3;

public class TLVec3 extends ThreadLocal<Vec3> {
	protected Vec3 initialValue(){
		return new Vec3();
	}
}
