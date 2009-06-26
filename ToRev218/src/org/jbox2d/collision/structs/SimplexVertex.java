package org.jbox2d.collision.structs;

import org.jbox2d.common.Vec2;

/**
 * GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
 * @author Daniel
 *
 */
public class SimplexVertex {
	public final Vec2 wA = new Vec2();		// support point in shapeA
	public final Vec2 wB = new Vec2();		// support point in shapeB
	public final Vec2 w = new Vec2();		// wB - wA
	public float a;		// barycentric coordinate for closest point
	public int indexA;	// wA index
	public int indexB;	// wB index
	
	public void set(SimplexVertex sv){
		wA.set(sv.wA);
		wB.set(sv.wB);
		w.set(sv.w);
		a = sv.a;
		indexA = sv.indexA;
		indexB = sv.indexB;
	}
}
