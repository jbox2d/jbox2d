package org.jbox2d.pooling;

import org.jbox2d.collision.AABB;

public class TLAABB extends CustThreadLocal<AABB> {
	protected AABB initialValue(){
		return new AABB();
	}
}
