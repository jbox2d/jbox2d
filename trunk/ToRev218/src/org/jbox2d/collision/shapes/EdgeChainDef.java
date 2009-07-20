package org.jbox2d.collision.shapes;

import java.util.LinkedList;

import org.jbox2d.collision.structs.ShapeType;
import org.jbox2d.common.Vec2;

/**
 * this structure is used to build edge shapes
 */
public class EdgeChainDef extends ShapeDef {
	
	/**
	 * The vertices in local coordinates.
	 */
	public final LinkedList<Vec2> vertices = new LinkedList<Vec2>();
	
	/**
	 * Whether to create an extra edge between the first and last vertices
	 */
	public boolean isALoop;
	
	public EdgeChainDef(){
		type = ShapeType.EDGE_SHAPE;
		isALoop = true;
	}
}
