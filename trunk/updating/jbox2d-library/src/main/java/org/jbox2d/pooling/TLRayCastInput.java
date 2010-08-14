package org.jbox2d.pooling;

import org.jbox2d.structs.collision.RayCastInput;

public class TLRayCastInput extends CustThreadLocal<RayCastInput> {
	protected RayCastInput initialValue(){
		return new RayCastInput();
	}
}
