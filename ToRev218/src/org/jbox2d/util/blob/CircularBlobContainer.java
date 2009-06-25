package org.jbox2d.util.blob;

import org.jbox2d.collision.AABB;
import org.jbox2d.common.Vec2;

/**
 * A circular blob container specified by radius and center.
 */
public class CircularBlobContainer implements BlobContainer {
	private float centerX, centerY;
	private float radius;
	private float radiusSqr;
	
	public CircularBlobContainer(Vec2 _center, float _radius) {
		centerX = _center.x;
		centerY = _center.y;
		radius = _radius;
		radiusSqr = _radius*_radius;
	}
	
	public float getRadius() {
		return radius;
	}
	
	public void setRadius(float r) {
		radius = r;
		radiusSqr = r*r;
	}
	
	public Vec2 getCenter() {
		return new Vec2(centerX,centerY);
	}
	
	public void setCenter(Vec2 c) {
		centerX = c.x;
		centerY = c.y;
	}
	
	public boolean containsPoint(Vec2 p) {
		float distSqr = (p.x-centerX)*(p.x-centerX)+(p.y-centerY)*(p.y-centerY);
		if (distSqr > radiusSqr) return false;
		return true;
	}

	public AABB getAABB() {
		Vec2 min = new Vec2(centerX-radius*1.2f,centerY-radius*1.2f);
		Vec2 max = new Vec2(centerX+radius*1.2f,centerY+radius*1.2f);
		return new AABB(min,max);
	}

}
