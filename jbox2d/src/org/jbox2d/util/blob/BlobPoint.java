package org.jbox2d.util.blob;

import org.jbox2d.common.Vec2;

public class BlobPoint {
	public Vec2 position;
	public float mass = 1.0f;
	
	public BlobPoint(float x, float y) {
		position = new Vec2(x,y);
	}
}
