package org.jbox2d.pooling;

import org.jbox2d.collision.BoundValues;

public class TLBoundValues extends ThreadLocal<BoundValues> {
	protected BoundValues initialValue(){
		return new BoundValues();
	}
}
