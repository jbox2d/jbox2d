package org.jbox2d.collision.shapes;

import org.jbox2d.common.Vec2;

/**
 * Point shape definition.
 */
public class PointDef extends ShapeDef {
	public Vec2 localPosition;
	public float mass;

	public PointDef() {
		type = ShapeType.POINT_SHAPE;
		localPosition = new Vec2(0.0f, 0.0f);
		mass = 0.0f;
	}
}
