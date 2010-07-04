package org.jbox2d.util.blob;

import org.jbox2d.collision.AABB;
import org.jbox2d.common.Vec2;

public interface BlobContainer {
	/**
	 * Is the Vec2 within the desired geometry?
	 * @param p The point to test
	 * @return True if the geometry contains the point
	 */
	public boolean containsPoint(Vec2 p);
	
	/** Get the world AABB of the container. */
	public AABB getAABB();
}
