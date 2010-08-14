package org.jbox2d.pooling;

import org.jbox2d.common.Transform;

public class TLTransform extends CustThreadLocal<Transform> {
	protected Transform initialValue(){
		return new Transform();
	}
}
