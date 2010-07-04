package org.jbox2d.pooling;

import org.jbox2d.collision.MassData;

public class TLMassData extends ThreadLocal<MassData> {
	protected MassData initialValue(){
		return new MassData();
	}
}
