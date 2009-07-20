/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.collision.handlers;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.Manifold.ManifoldType;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

//Updated to rev 139 of b2CollideCircle.cpp

/**
 * Circle/circle and circle/polygon overlap solver -
 * for internal use only.
 */
public class CollideCircle {

	// djm pooled
	private final static Vec2 colCCP1 = new Vec2();
	private final static Vec2 colCCP2 = new Vec2();
	private final static Vec2 colCCD = new Vec2();
	//private final static Vec2 colCCP = new Vec2();

	/**
	 * puts collision information of the two circles in the manifold
	 * @param manifold
	 * @param circle1
	 * @param xf1
	 * @param circle2
	 * @param xf2
	 */
	// djm optimized
	public final static void collideCircles(final Manifold manifold,
	                                        final CircleShape circle1, final XForm xf1,
	                                        final CircleShape circle2, final XForm xf2) {
		manifold.m_pointCount = 0;

		XForm.mulToOut(xf1, circle1.m_localPosition, colCCP1);
		XForm.mulToOut(xf2, circle2.m_localPosition, colCCP2);

		colCCD.x = colCCP2.x - colCCP1.x;
		colCCD.y = colCCP2.y - colCCP1.y;

		final float distSqr = Vec2.dot(colCCD, colCCD);

		//final float r1 = circle1.m_radius;
		//final float r2 = circle2.m_radius;
		final float radiusSum =  circle1.m_radius + circle2.m_radius;
		if (distSqr > radiusSum * radiusSum) {
			return;
		}
		
		// MB these sets
		manifold.m_type = ManifoldType.e_circles;
		manifold.m_localPoint.set(circle1.m_localPosition);
		manifold.m_localPlaneNormal.setZero();
		manifold.m_pointCount = 1;
		
		manifold.m_points[0].m_localPoint.set(circle2.m_localPosition);
		manifold.m_points[0].m_id.key = 0;
		
		/*float separation;
		if (distSqr < Settings.EPSILON) {
			separation = -radiusSum;
			manifold.m_normal.set(0.0f, 1.0f);
		}
		else {
			final float dist = (float) Math.sqrt(distSqr);
			separation = dist - radiusSum;
			final float a = 1.0f / dist;
			manifold.m_normal.x = a * colCCD.x;
			manifold.m_normal.y = a * colCCD.y;
		}

		manifold.m_pointCount = 1;
		//manifold.m_points[0].id.key = 0;
		manifold.m_points[0].id.zero(); //use this instead of zeroing through key
		manifold.m_points[0].separation = separation;

		// let this one slide
		colCCP1.addLocal(manifold.m_normal.mul(r1));
		colCCP2.subLocal(manifold.m_normal.mul(r2));

		colCCP.x = 0.5f * (colCCP1.x + colCCP2.x);
		colCCP.y = 0.5f * (colCCP1.y + colCCP2.y);

		XForm.mulTransToOut(xf1, colCCP, manifold.m_points[0].localPoint1);
		XForm.mulTransToOut(xf2, colCCP, manifold.m_points[0].localPoint2);*/
	}
	
	private final static Vec2 temp1 = new Vec2();
	private final static Vec2 temp2 = new Vec2();
	/**
	 * puts collision information about the collision of a polygon and a circle
	 * @param manifold
	 * @param polygon
	 * @param xf1
	 * @param circle
	 * @param xf2
	 */
	public final static void collidePolygonAndCircle(final Manifold manifold,
	                                                 final PolygonShape polygon, final XForm xf1,
	                                                 final CircleShape circle, final XForm xf2) {

		manifold.m_pointCount = 0;

		// Compute circle position in the frame of the polygon.
		// INLINED
		//Vec2 c = XForm.mul(xf2, circle.getLocalPosition());
		//Vec2 cLocal = XForm.mulT(xf1, c);

		final float cx = xf2.position.x + xf2.R.col1.x * circle.m_localPosition.x + xf2.R.col2.x * circle.m_localPosition.y;
		final float cy = xf2.position.y + xf2.R.col1.y * circle.m_localPosition.x + xf2.R.col2.y * circle.m_localPosition.y;
		final float v1x = cx - xf1.position.x;
		final float v1y = cy - xf1.position.y;
		final float cLocalx = v1x * xf1.R.col1.x + v1y * xf1.R.col1.y;
		final float cLocaly = v1x * xf1.R.col2.x + v1y * xf1.R.col2.y;

		// Find edge with maximum separation.
		int normalIndex = 0;
		float separation = -Float.MAX_VALUE;
		final float radius = circle.m_radius + polygon.m_radius;
		final int vertexCount = polygon.getVertexCount();
		final Vec2[] vertices = polygon.getVertices();
		final Vec2[] normals = polygon.getNormals();
		
		for (int i = 0; i < vertexCount; ++i) {

			// INLINED
			//float s = Vec2.dot(normals[i], cLocal.sub(vertices[i]));
			final float s = normals[i].x * (cLocalx - vertices[i].x) + normals[i].y * (cLocaly - vertices[i].y);

			if (s > circle.m_radius) {
				// Early out.
				return;
			}

			if (s > separation) {
				normalIndex = i;
				separation = s;
			}
		}
		
		int vertIndex1 = normalIndex;
		int vertIndex2 = (vertIndex1 + 1 < vertexCount) ? vertIndex1 + 1 : 0;
		Vec2 v1 = vertices[vertIndex1];
		Vec2 v2 = vertices[vertIndex2];
		
		// If the center is inside the polygon ...
		if (separation < Settings.EPSILON) {
			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			manifold.m_localPlaneNormal.set(normals[normalIndex]);
			manifold.m_localPoint.set(v1).addLocal(v2).mulLocal(.5f);
			
			manifold.m_points[0].m_localPoint.set(circle.m_localPosition);
			manifold.m_points[0].m_id.key = 0;
			return;
			// INLINED
			//Vec2 position = c.sub(manifold.m_normal.mul(radius));
			//manifold.m_points[0].localPoint1 = XForm.mulT(xf1, position);
			//manifold.m_points[0].localPoint2 = XForm.mulT(xf2, position);

			/*final float positionx = cx - manifold.m_normal.x * radius;
			final float positiony = cy - manifold.m_normal.y * radius;
			final float v1x1 = positionx-xf1.position.x;
			final float v1y1 = positiony-xf1.position.y;
			manifold.m_points[0].localPoint1.x = (v1x1 * xf1.R.col1.x + v1y1 * xf1.R.col1.y);
			manifold.m_points[0].localPoint1.y = (v1x1 * xf1.R.col2.x + v1y1 * xf1.R.col2.y);
			final float v1x2 = positionx-xf2.position.x;
			final float v1y2 = positiony-xf2.position.y;
			manifold.m_points[0].localPoint2.x = (v1x2 * xf2.R.col1.x + v1y2 * xf2.R.col1.y);
			manifold.m_points[0].localPoint2.y = (v1x2 * xf2.R.col2.x + v1y2 * xf2.R.col2.y);

			manifold.m_points[0].separation = separation - radius;
			return;*/
		}

		// Project the circle center onto the edge segment.
		/*final int vertIndex1 = normalIndex;
		final int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		// INLINED
		//Vec2 e = vertices[vertIndex2].sub(vertices[vertIndex1]);
		//float length = e.normalize();

		float ex = vertices[vertIndex2].x - vertices[vertIndex1].x;
		float ey = vertices[vertIndex2].y - vertices[vertIndex1].y;
		final float length = (float) Math.sqrt(ex * ex + ey * ey);
		assert(length > Settings.EPSILON);
		final float invLength = 1.0f / length;
		ex *= invLength;
		ey *= invLength;

		// Project the center onto the edge.
		// INLINED
		//float u = Vec2.dot(cLocal.sub(vertices[vertIndex1]), e);
		final float u = (cLocalx - vertices[vertIndex1].x) * ex + (cLocaly - vertices[vertIndex1].y) * ey;

		float px;
		float py;*/
		
		// Compute barycentric coordinates
		temp1.set(cLocalx, cLocaly).subLocal(v1);
		temp2.set(v2).subLocal(v1);
		float u1 = Vec2.dot(temp1, temp2);
		
		temp1.set(cLocalx, cLocaly).subLocal(v2);
		temp2.set(v1).subLocal(v2);
		float u2 = Vec2.dot(temp1, temp2);
		if (u1 <= 0.0f) {
			temp1.set(cLocalx,cLocaly);
			if(MathUtils.distanceSquared(temp1, v1) > radius * radius){
				return;
			}
			
			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			manifold.m_localPlaneNormal.set(cLocalx,cLocaly).subLocal(v1);
			manifold.m_localPlaneNormal.normalize();
			manifold.m_localPoint.set(v1);
			manifold.m_points[0].m_localPoint.set(circle.m_localPosition);
			manifold.m_points[0].m_id.key = 0;
		}
		else if (u2 <= 0.0f) {
			temp1.set(cLocalx,cLocaly);
			
			if(MathUtils.distanceSquared(temp1, v2) > radius * radius){
				return;
			}

			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			manifold.m_localPlaneNormal.set(cLocalx,cLocaly).subLocal(v2);
			manifold.m_localPlaneNormal.normalize();
			manifold.m_localPoint.set(v2);
			manifold.m_points[0].m_localPoint.set(circle.m_localPosition);
			manifold.m_points[0].m_id.key = 0;
		}
		else {
			//b2Vec2 faceCenter = 0.5f * (v1 + v2);
			temp1.set(v1).addLocal(v2).mulLocal(.5f);
			temp2.set(cLocalx,cLocaly).subLocal(temp1);
			separation = Vec2.dot(temp2, normals[vertIndex1]);
			if (separation > radius){
				return;
			}

			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			manifold.m_localPlaneNormal.set(normals[vertIndex1]);
			manifold.m_localPoint.set(temp1);
			manifold.m_points[0].m_localPoint.set(circle.m_localPosition);
			manifold.m_points[0].m_id.key = 0;
		}

		// INLINED
		//Vec2 d = cLocal.sub(p);
		//float dist = d.normalize();

		/*float dx = cLocalx - px;
		float dy = cLocaly - py;
		final float dist = (float) Math.sqrt(dx * dx + dy * dy);
		if (dist > radius) {
			return;
		}
		if (dist >= Settings.EPSILON) {
			final float invDist = 1.0f / dist;
			dx *= invDist;
			dy *= invDist;
		}

		manifold.m_pointCount = 1;

		// INLINED
		//manifold.m_normal = Mat22.mul(xf1.R, d);
		//Vec2 position = c.sub(manifold.m_normal.mul(radius));
		//manifold.m_points[0].localPoint1 = XForm.mulT(xf1, position);
		//manifold.m_points[0].localPoint2 = XForm.mulT(xf2, position);

		manifold.m_normal.x = xf1.R.col1.x * dx + xf1.R.col2.x * dy;
		manifold.m_normal.y = xf1.R.col1.y * dx + xf1.R.col2.y * dy;
		final float positionx = cx - manifold.m_normal.x * radius;
		final float positiony = cy - manifold.m_normal.y * radius;
		final float v1x1 = positionx - xf1.position.x;
		final float v1y1 = positiony - xf1.position.y;
		manifold.m_points[0].localPoint1.x = (v1x1 * xf1.R.col1.x + v1y1 * xf1.R.col1.y);
		manifold.m_points[0].localPoint1.y = (v1x1 * xf1.R.col2.x + v1y1 * xf1.R.col2.y);
		final float v1x2 = positionx - xf2.position.x;
		final float v1y2 = positiony - xf2.position.y;
		manifold.m_points[0].localPoint2.x = (v1x2 * xf2.R.col1.x + v1y2 * xf2.R.col1.y);
		manifold.m_points[0].localPoint2.y = (v1x2 * xf2.R.col2.x + v1y2 * xf2.R.col2.y);

		manifold.m_points[0].separation = dist - radius;
		manifold.m_points[0].id.features.referenceEdge = 0;
		manifold.m_points[0].id.features.flip = 0;*/
	}
}