package org.jbox2d.collision;

public interface QueryCallback {

	public void queryCallback(AABB aabb, Object userData);
}
