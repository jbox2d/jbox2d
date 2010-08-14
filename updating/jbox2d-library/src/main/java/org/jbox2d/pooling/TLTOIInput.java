package org.jbox2d.pooling;

import org.jbox2d.structs.collision.TOIInput;

public class TLTOIInput extends CustThreadLocal<TOIInput> {
	protected TOIInput initialValue(){
		return new TOIInput();
	}
}
