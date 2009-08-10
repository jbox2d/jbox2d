package org.jbox2d.pooling;

import org.jbox2d.common.Mat33;

public class TLMat33 extends ThreadLocal<Mat33> {
	protected Mat33 initialValue(){
		return new Mat33();
	}
}
