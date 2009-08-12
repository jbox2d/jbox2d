package org.jbox2d.pooling;

import org.jbox2d.structs.collision.broadphase.BroadPhaseQueryWrapper;

public class TLBroadPhaseQueryWrapper extends ThreadLocal<BroadPhaseQueryWrapper> {
	protected BroadPhaseQueryWrapper initialValue(){
		return new BroadPhaseQueryWrapper();
	}
}
