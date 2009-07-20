package org.jbox2d.collision.shapes;

import org.jbox2d.collision.structs.ShapeType;
import org.jbox2d.common.Vec2;

public class CircleDef extends ShapeDef {
	
	public Vec2 localPosition;
	public float radius;
	
	public CircleDef(){
		type = ShapeType.CIRCLE_SHAPE;
		localPosition = new Vec2();
		radius = 1.0f;
	}
}
