package org.jbox2d.pooling;

import org.jbox2d.collision.shapes.MassData;

public class TLMassData extends CustThreadLocal<MassData> {
	protected MassData initialValue(){
		return new MassData();
	}
}
