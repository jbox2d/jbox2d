package org.jbox2d.pooling;

import org.jbox2d.dynamics.FixtureDef;

public class TLFixtureDef extends CustThreadLocal<FixtureDef> {
	protected final FixtureDef initialValue(){
		return new FixtureDef();
	}
}
