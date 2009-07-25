package org.jbox2d.pooling;

import org.jbox2d.common.Mat22;

public class ThreadLocalMat22 extends ThreadLocal<Mat22> {
	protected Mat22 initialValue() {
		return new Mat22();
	}
}
