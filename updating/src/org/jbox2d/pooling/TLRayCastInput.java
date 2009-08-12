package org.jbox2d.pooling;

import org.jbox2d.structs.collision.RayCastInput;

public class TLRayCastInput extends ThreadLocal<RayCastInput> {
	protected RayCastInput initialValue(){
		return new RayCastInput();
	}
}
