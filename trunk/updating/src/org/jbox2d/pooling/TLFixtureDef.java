package org.jbox2d.pooling;

import org.jbox2d.dynamics.FixtureDef;

public class TLFixtureDef extends ThreadLocal<FixtureDef> {
	protected final FixtureDef initialValue(){
		return new FixtureDef();
	}
}
