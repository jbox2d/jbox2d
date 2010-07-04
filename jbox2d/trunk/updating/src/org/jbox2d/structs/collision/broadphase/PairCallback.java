package org.jbox2d.structs.collision.broadphase;

public interface PairCallback {
	public void addPair(Object userDataA, Object userDataB);
}
