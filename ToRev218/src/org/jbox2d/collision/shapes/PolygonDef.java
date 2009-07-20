package org.jbox2d.collision.shapes;

import org.jbox2d.collision.structs.ShapeType;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

/**
 * Convex polygon. The vertices must be in CCW order for a right-handed
 * coordinate system with the z-axis coming out of the screen.
 *
 */
public class PolygonDef extends ShapeDef{
	
	public final Vec2[] vertices = new Vec2[Settings.maxPolygonVertices];
	public int vertexCount;
	
	public PolygonDef(){
		type = ShapeType.POLYGON_SHAPE;
		vertexCount = 0;
	}
	
	/**
	 * Build vertices to represent an axis-aligned box.
	 * @param hx the half-width.
	 * @param hy the half-height.
	 */
	public void setAsBox(float hx, float hy){
		vertexCount = 4;
		vertices[0].set(-hx, -hy);
		vertices[1].set( hx, -hy);
		vertices[2].set( hx,  hy);
		vertices[3].set(-hx,  hy);
	}

	// djm pooled
	private static final XForm xf = new XForm();
	/**
	 * Build vertices to represent an oriented box.
	 * @param hx the half-width.
	 * @param hy the half-height.
	 * @param center the center of the box in local coordinates.
	 * @param angle the rotation of the box in local coordinates.
	 */
	public void setAsBox(float hx, float hy, Vec2 center, float angle){
		setAsBox(hx, hy);
		xf.position = center;
		xf.R.set(angle);

		for (int i = 0; i < vertexCount; ++i){
			XForm.mulToOut( xf, vertices[i], vertices[i]);
		}
	}
}
