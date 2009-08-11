package org.jbox2d.collision;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.TLVec2;
import org.jbox2d.structs.ContactID;
import org.jbox2d.structs.Manifold;
import org.jbox2d.structs.PointState;
import org.jbox2d.structs.Manifold.ManifoldType;

/**
 * Functions used for computing contact points, distance
 * queries, and TOI queries.
 *
 * @author daniel
 */
public class Collision {
	public static final int NULL_FEATURE = Integer.MAX_VALUE;
	
	/**
	 * Compute the point states given two manifolds. The states pertain to the transition from manifold1
	 * to manifold2. So state1 is either persist or remove while state2 is either add or persist.
	 * @param state1
	 * @param state2
	 * @param manifold1
	 * @param manifold2
	 */
	public static final void getPointStates(final PointState[] state1, final PointState[] state2,
	                                        final Manifold manifold1, final Manifold manifold2){
		assert(state1.length == Settings.maxManifoldPoints);
		assert(state2.length == Settings.maxManifoldPoints);
		
		for( int i=0; i< Settings.maxManifoldPoints; i++){
			state1[i] = PointState.NULL_STATE;
			state2[i] = PointState.NULL_STATE;
		}
		
		// Detect persists and removes.
		for( int i=0; i<manifold1.m_pointCount; i++){
			ContactID id = manifold1.m_points[i].m_id;
			
			state1[i] = PointState.REMOVE_STATE;
			
			for( int j = 0; j < manifold2.m_pointCount; j++){
				if(manifold2.m_points[j].m_id.key == id.key){
					state1[i] = PointState.PERSIST_STATE;
					break;
				}
			}
		}
		
		// Detect persists and adds
		for( int i=0; i< manifold2.m_pointCount; i++){
			ContactID id = manifold2.m_points[i].m_id;
			
			state2[i] = PointState.ADD_STATE;
			
			for(int j=0; j < manifold1.m_pointCount; j++){
				if(manifold1.m_points[j].m_id.key == id.key){
					state2[i] = PointState.PERSIST_STATE;
					break;
				}
			}
		}
	}
	
	// djm pooling
	private static final TLVec2 tlp1 = new TLVec2();
	private static final TLVec2 tlp2 = new TLVec2();
	private static final TLVec2 tld = new TLVec2();

	/**
	 * Compute the collision manifold between two circles.
	 * @param manifold
	 * @param circle1
	 * @param xf1
	 * @param circle2
	 * @param xf2
	 */
	public static final void collideCircles(Manifold manifold, final CircleShape circle1, final Transform xf1,
	                                        final CircleShape circle2, final Transform xf2){
		manifold.m_pointCount = 0;
		
		final Vec2 p1 = tlp1.get();
		final Vec2 p2 = tlp2.get();
		final Vec2 d = tld.get();
		
		Transform.mulToOut( xf1, circle1.m_p, p1);
		Transform.mulToOut( xf2, circle2.m_p, p2);
		d.set(p2).subLocal(p1);
		
		float distSqr = Vec2.dot(d, d);
		float radius = circle1.m_radius + circle2.m_radius;
		if( distSqr > radius * radius){
			return;
		}
		
		manifold.m_type = ManifoldType.e_circles;
		manifold.m_localPoint.set(circle1.m_p);
		manifold.m_localPlaneNormal.setZero();
		manifold.m_pointCount = 1;
		
		manifold.m_points[0].m_localPoint.set(circle2.m_p);
		manifold.m_points[0].m_id.key = 0;
	}
	
	// djm pooling, and from above
	private static final TLVec2 tlc = new TLVec2();
	private static final TLVec2 tlcLocal = new TLVec2();
	private static final TLVec2 tltemp = new TLVec2();
	private static final TLVec2 tltemp2 = new TLVec2();
	/**
	 * Compute the collision manifold between a polygon and a circle.
	 * @param manifold
	 * @param polygon
	 * @param xf1
	 * @param circle
	 * @param xf2
	 */
	public static final void collidePolygonAndCircle(Manifold manifold, final PolygonShape polygon, final Transform xf1,
	                                        final CircleShape circle, final Transform xf2){
		manifold.m_pointCount = 0;
		
		final Vec2 c = tlc.get();
		final Vec2 cLocal = tlcLocal.get();
		final Vec2 temp = tltemp.get();
		
		// Compute circle position in the frame of the polygon.
		Transform.mulToOut(xf2, circle.m_p, c);
		Transform.mulToOut(xf1, c, cLocal);
		
		// Find the min separating edge.
		int normalIndex = 0;
		float separation = Float.MIN_VALUE;
		float radius = polygon.m_radius+circle.m_radius;
		int vertexCount = polygon.m_vertexCount;
		
		Vec2[] vertices = polygon.m_vertices;
		Vec2[] normals = polygon.m_normals;
		
		for(int i=0; i < vertexCount; i++){
			temp.set(cLocal).subLocal(vertices[i]);
			float s = Vec2.dot( normals[i], temp);
			
			if(s > radius){
				// early out
				return;
			}
			
			if(s > separation){
				separation = s;
				normalIndex = i;
			}
		}
		
		// Vertices that subtend the incident face.
		int vertIndex1 = normalIndex;
		int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		Vec2 v1 = vertices[vertIndex1];
		Vec2 v2 = vertices[vertIndex2];
		
		// If the center is inside the polygon ...
		if(separation < Settings.EPSILON){
			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			manifold.m_localPlaneNormal.set(normals[normalIndex]);
			manifold.m_localPoint.set( v1).addLocal(v2).mulLocal( .5f);
			manifold.m_points[0].m_localPoint.set(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
			return;
		}
		
		final Vec2 temp2 = tltemp2.get();
		
		// Compute barycentric coordinates
		temp.set(cLocal).subLocal(v1);
		temp2.set(v2).subLocal(v1);
		float u1 = Vec2.dot(temp, temp2);
		temp.set(cLocal).subLocal(v2);
		temp2.set(v1).subLocal(v2);
		float u2 = Vec2.dot(temp, temp2);
		
		if(u1 <= 0f){
			if(MathUtils.distanceSquared(cLocal, v1) > radius * radius){
				return;
			}
			
			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			manifold.m_localPlaneNormal.set(cLocal).subLocal(v1);
			manifold.m_localPlaneNormal.normalize();
			manifold.m_localPoint.set(v1);
			manifold.m_points[0].m_localPoint.set(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
		else if (u2 <= 0.0f){
			if(MathUtils.distanceSquared( cLocal, v2) > radius * radius){
				return;
			}
			
			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			manifold.m_localPlaneNormal.set(cLocal).subLocal(v2);
			manifold.m_localPlaneNormal.normalize();
			manifold.m_localPoint.set(v2);
			manifold.m_points[0].m_localPoint.set(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
		else {
			//Vec2 faceCenter = 0.5f * (v1 + v2);
			temp.set(v1).addLocal(v2).mulLocal(.5f);
			
			temp2.set(cLocal).subLocal(temp);
			separation = Vec2.dot(temp2, normals[vertIndex1]);
			if (separation > radius){
				return;
			}

			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			manifold.m_localPlaneNormal.set(normals[vertIndex1]);
			manifold.m_localPoint.set(temp);
			manifold.m_points[0].m_localPoint.set(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
	}
	
	// djm pooling
	private static final TLVec2 tlnormal1World = new TLVec2();
	private static final TLVec2 tlnormal1 = new TLVec2();
	private static final TLVec2 tlv1 = new TLVec2();
	private static final TLVec2 tlv2 = new TLVec2();

	/**
	 * Find the separation between poly1 and poly2 for a given edge normal on poly1.
	 * @param poly1
	 * @param xf1
	 * @param edge1
	 * @param poly2
	 * @param xf2
	 */
	private static final float edgeSeparation( final PolygonShape poly1, final Transform xf1, final int edge1,
	                                          final PolygonShape poly2, final Transform xf2){
		int count1 = poly1.m_vertexCount;
		final Vec2[] vertices1 = poly1.m_vertices;
		final Vec2[] normals1 = poly1.m_normals;

		int count2 = poly2.m_vertexCount;
		final Vec2[] vertices2 = poly2.m_vertices;

		assert(0 <= edge1 && edge1 < count1);

		final Vec2 normal1World = tlnormal1World.get();
		final Vec2 normal1 = tlnormal1.get();
		
		// Convert normal from poly1's frame into poly2's frame.
		//Vec2 normal1World = Mul(xf1.R, normals1[edge1]);
		Mat22.mulToOut( xf1.R, normals1[edge1], normal1World);
		//Vec2 normal1 = MulT(xf2.R, normal1World);
		Mat22.mulToOut( xf2.R, normal1World, normal1);

		// Find support vertex on poly2 for -normal.
		int index = 0;
		float minDot = Float.MAX_VALUE;

		for (int i = 0; i < count2; ++i){
			float dot = Vec2.dot(vertices2[i], normal1);
			if (dot < minDot){
				minDot = dot;
				index = i;
			}
		}

		final Vec2 v1 = tlv1.get();
		final Vec2 v2 = tlv2.get();
		
		//Vec2 v1 = Mul(xf1, vertices1[edge1]);
		//Vec2 v2 = Mul(xf2, vertices2[index]);
		Transform.mulToOut( xf1, vertices1[edge1], v1);
		Transform.mulToOut( xf2, vertices2[index], v2);
		
		float separation = Vec2.dot(v2.subLocal(v1), normal1World);
		return separation;
	}
	/**
	 * Compute the collision manifold between two polygons.
	 * @param manifold
	 * @param polygon1
	 * @param xf1
	 * @param polygon2
	 * @param xf2
	 */
	public static final void collidePolygons(Manifold manifold, final PolygonShape polygon1, final Transform xf1,
	                                        final PolygonShape polygon2, final Transform xf2){
		
	}
	
	/**
	 * Clipping for contact manifolds.
	 * Sutherland-Hodgman clipping.
	 * @param vOut
	 * @param vIn
	 * @param normal
	 * @param offset
	 * @return
	 */
	public static final int clipSegmentToLine(final ClipVertex[] vOut, final ClipVertex[] vIn, final Vec2 normal, float offset){
		assert(vOut.length == 2);
		assert(vIn.length == 2);
		
		// Start with no output points
		int numOut = 0;
		
		// Calculate the distance of end points to the line
		float distance0 = Vec2.dot( normal, vIn[0].v) - offset;
		float distance1 = Vec2.dot( normal, vIn[1].v) - offset;
		
		// If the points are behind the plane
		if (distance0 <= 0.0f){
			vOut[numOut++] = vIn[0];
		}
		if (distance1 <= 0.0f){
			vOut[numOut++] = vIn[1];
		}
		
		// If the points are on different sides of the plane
		if (distance0 * distance1 < 0.0f){
			// Find intersection point of edge and plane
			float interp = distance0 / (distance0 - distance1);
			//vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
			vOut[numOut].v.set(vIn[1].v).subLocal(vIn[0].v).mulLocal( interp).addLocal( vIn[0].v);
			if (distance0 > 0.0f){
				vOut[numOut].id.set(vIn[0].id);
			}
			else{
				vOut[numOut].id.set(vIn[1].id);
			}
			++numOut;
		}

		return numOut;
	}
}
