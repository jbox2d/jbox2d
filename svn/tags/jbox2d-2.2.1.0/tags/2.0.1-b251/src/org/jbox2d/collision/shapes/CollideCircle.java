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

package org.jbox2d.collision.shapes;

import org.jbox2d.collision.Collision;
import org.jbox2d.collision.Manifold;
import org.jbox2d.common.Mat22;
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
	private final Vec2 colCCP1 = new Vec2();
	private final Vec2 colCCP2 = new Vec2();
	private final Vec2 colCCD = new Vec2();
	private final Vec2 colCCP = new Vec2();

	/**
	 * puts collision information of the two circles in the manifold
	 * @param manifold
	 * @param circle1
	 * @param xf1
	 * @param circle2
	 * @param xf2
	 */
	// djm optimized
	public final void collideCircles(final Manifold manifold,
	                                        final CircleShape circle1, final XForm xf1,
	                                        final CircleShape circle2, final XForm xf2) {
		manifold.pointCount = 0;

		XForm.mulToOut(xf1, circle1.getMemberLocalPosition(), colCCP1);
		XForm.mulToOut(xf2, circle2.getMemberLocalPosition(), colCCP2);

		colCCD.x = colCCP2.x - colCCP1.x;
		colCCD.y = colCCP2.y - colCCP1.y;

		final float distSqr = Vec2.dot(colCCD, colCCD);

		final float r1 = circle1.getRadius();
		final float r2 = circle2.getRadius();
		final float radiusSum = r1+r2;
		if (distSqr > radiusSum * radiusSum) {
			return;
		}

		float separation;
		if (distSqr < Settings.EPSILON) {
			separation = -radiusSum;
			manifold.normal.set(0.0f, 1.0f);
		}
		else {
			final float dist = MathUtils.sqrt(distSqr);
			separation = dist - radiusSum;
			final float a = 1.0f / dist;
			manifold.normal.x = a * colCCD.x;
			manifold.normal.y = a * colCCD.y;
		}

		manifold.pointCount = 1;
		//manifold.points[0].id.key = 0;
		manifold.points[0].id.zero(); //use this instead of zeroing through key
		manifold.points[0].separation = separation;

		// let this one slide
		colCCP1.addLocal(manifold.normal.mul(r1));
		colCCP2.subLocal(manifold.normal.mul(r2));

		colCCP.x = 0.5f * (colCCP1.x + colCCP2.x);
		colCCP.y = 0.5f * (colCCP1.y + colCCP2.y);

		XForm.mulTransToOut(xf1, colCCP, manifold.points[0].localPoint1);
		XForm.mulTransToOut(xf2, colCCP, manifold.points[0].localPoint2);
	}

	// djm pooled
	private final Vec2 colPCP1 = new Vec2();
	private final Vec2 colPCP2 = new Vec2();
	private final Vec2 colPCD = new Vec2();
	private final Vec2 colPCP = new Vec2();

	/**
	 * Puts collision information in the manifold about a collision between a point and a circle
	 * @param manifold
	 * @param point1
	 * @param xf1
	 * @param circle2
	 * @param xf2
	 */
	// djm optimized
	public final void collidePointAndCircle(final Manifold manifold,
	                                               final PointShape point1, final XForm xf1,
	                                               final CircleShape circle2, final XForm xf2) {
		manifold.pointCount = 0;

		XForm.mulToOut(xf1, point1.getMemberLocalPosition(), colPCP1);
		XForm.mulToOut(xf2, circle2.getMemberLocalPosition(), colPCP2);

		colPCD.x = colPCP2.x - colPCP1.x;
		colPCD.y = colPCP2.y - colPCP1.y;

		final float distSqr = Vec2.dot(colPCD, colPCD);

		final float r2 = circle2.getRadius();

		if (distSqr > r2*r2) {
			return;
		}

		float separation;
		if (distSqr < Settings.EPSILON) {
			separation = -r2;
			manifold.normal.set(0.0f, 1.0f);
		}
		else {
			final float dist = MathUtils.sqrt(distSqr);
			separation = dist - r2;
			final float a = 1.0f / dist;
			manifold.normal.x = a * colPCD.x;
			manifold.normal.y = a * colPCD.y;
		}

		manifold.pointCount = 1;
		//manifold.points[0].id.key = 0;
		manifold.points[0].id.zero(); //use this instead of zeroing through key
		manifold.points[0].separation = separation;

		// leave this for now
		colPCP2.subLocal(manifold.normal.mul(r2));

		colPCP.x = 0.5f * (colPCP1.x + colPCP2.x);
		colPCP.y = 0.5f * (colPCP1.y + colPCP2.y);

		XForm.mulTransToOut(xf1, colPCP, manifold.points[0].localPoint1);
		XForm.mulTransToOut(xf2, colPCP, manifold.points[0].localPoint2);
	}

	/**
	 * puts collision information about the collision of a polygon and a circle
	 * @param manifold
	 * @param polygon
	 * @param xf1
	 * @param circle
	 * @param xf2
	 */
	public final void collidePolygonAndCircle(final Manifold manifold,
	                                                 final PolygonShape polygon, final XForm xf1,
	                                                 final CircleShape circle, final XForm xf2) {

		manifold.pointCount = 0;

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
		final float radius = circle.getRadius();
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
		// If the center is inside the polygon ...
		if (separation < Settings.EPSILON) {
			manifold.pointCount = 1;
			// INLINED
			//manifold.normal = Mat22.mul(xf1.R, normals[normalIndex]);
			manifold.normal.x = xf1.R.col1.x * normals[normalIndex].x + xf1.R.col2.x * normals[normalIndex].y;
			manifold.normal.y = xf1.R.col1.y * normals[normalIndex].x + xf1.R.col2.y * normals[normalIndex].y;

			manifold.points[0].id.features.incidentEdge = normalIndex;
			manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
			manifold.points[0].id.features.referenceEdge = 0;
			manifold.points[0].id.features.flip = 0;
			// INLINED
			//Vec2 position = c.sub(manifold.normal.mul(radius));
			//manifold.points[0].localPoint1 = XForm.mulT(xf1, position);
			//manifold.points[0].localPoint2 = XForm.mulT(xf2, position);

			final float positionx = cx - manifold.normal.x * radius;
			final float positiony = cy - manifold.normal.y * radius;
			final float v1x1 = positionx-xf1.position.x;
			final float v1y1 = positiony-xf1.position.y;
			manifold.points[0].localPoint1.x = (v1x1 * xf1.R.col1.x + v1y1 * xf1.R.col1.y);
			manifold.points[0].localPoint1.y = (v1x1 * xf1.R.col2.x + v1y1 * xf1.R.col2.y);
			final float v1x2 = positionx-xf2.position.x;
			final float v1y2 = positiony-xf2.position.y;
			manifold.points[0].localPoint2.x = (v1x2 * xf2.R.col1.x + v1y2 * xf2.R.col1.y);
			manifold.points[0].localPoint2.y = (v1x2 * xf2.R.col2.x + v1y2 * xf2.R.col2.y);

			manifold.points[0].separation = separation - radius;
			return;
		}

		// Project the circle center onto the edge segment.
		final int vertIndex1 = normalIndex;
		final int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		// INLINED
		//Vec2 e = vertices[vertIndex2].sub(vertices[vertIndex1]);
		//float length = e.normalize();

		float ex = vertices[vertIndex2].x - vertices[vertIndex1].x;
		float ey = vertices[vertIndex2].y - vertices[vertIndex1].y;
		final float length = MathUtils.sqrt(ex * ex + ey * ey);
		assert(length > Settings.EPSILON);
		final float invLength = 1.0f / length;
		ex *= invLength;
		ey *= invLength;

		// Project the center onto the edge.
		// INLINED
		//float u = Vec2.dot(cLocal.sub(vertices[vertIndex1]), e);
		final float u = (cLocalx - vertices[vertIndex1].x) * ex + (cLocaly - vertices[vertIndex1].y) * ey;

		float px;
		float py;
		if (u <= 0.0f) {
			px = vertices[vertIndex1].x;
			py = vertices[vertIndex1].y;
			manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
			manifold.points[0].id.features.incidentVertex = vertIndex1;
		}
		else if (u >= length) {
			px = vertices[vertIndex2].x;
			py = vertices[vertIndex2].y;
			manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
			manifold.points[0].id.features.incidentVertex = vertIndex2;
		}
		else {
			px = vertices[vertIndex1].x;
			py = vertices[vertIndex1].y;
			px += u * ex;
			py += u * ey;
			manifold.points[0].id.features.incidentEdge = normalIndex;
			manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
		}

		// INLINED
		//Vec2 d = cLocal.sub(p);
		//float dist = d.normalize();

		float dx = cLocalx - px;
		float dy = cLocaly - py;
		final float dist = MathUtils.sqrt(dx * dx + dy * dy);
		if (dist > radius) {
			return;
		}
		if (dist >= Settings.EPSILON) {
			final float invDist = 1.0f / dist;
			dx *= invDist;
			dy *= invDist;
		}

		manifold.pointCount = 1;

		// INLINED
		//manifold.normal = Mat22.mul(xf1.R, d);
		//Vec2 position = c.sub(manifold.normal.mul(radius));
		//manifold.points[0].localPoint1 = XForm.mulT(xf1, position);
		//manifold.points[0].localPoint2 = XForm.mulT(xf2, position);

		manifold.normal.x = xf1.R.col1.x * dx + xf1.R.col2.x * dy;
		manifold.normal.y = xf1.R.col1.y * dx + xf1.R.col2.y * dy;
		final float positionx = cx - manifold.normal.x * radius;
		final float positiony = cy - manifold.normal.y * radius;
		final float v1x1 = positionx - xf1.position.x;
		final float v1y1 = positiony - xf1.position.y;
		manifold.points[0].localPoint1.x = (v1x1 * xf1.R.col1.x + v1y1 * xf1.R.col1.y);
		manifold.points[0].localPoint1.y = (v1x1 * xf1.R.col2.x + v1y1 * xf1.R.col2.y);
		final float v1x2 = positionx - xf2.position.x;
		final float v1y2 = positiony - xf2.position.y;
		manifold.points[0].localPoint2.x = (v1x2 * xf2.R.col1.x + v1y2 * xf2.R.col1.y);
		manifold.points[0].localPoint2.y = (v1x2 * xf2.R.col2.x + v1y2 * xf2.R.col2.y);

		manifold.points[0].separation = dist - radius;
		manifold.points[0].id.features.referenceEdge = 0;
		manifold.points[0].id.features.flip = 0;
	}

	private final Vec2 ECd = new Vec2();
	private final Vec2 ECc = new Vec2();
	//private Vec2 ECcLocalSubV1 = new Vec2();
	private final Vec2 ECcLocal = new Vec2();

	/**
	 * puts collision information into the manifold from a circle and edge collision
	 * @param manifold
	 * @param edge
	 * @param xf1
	 * @param circle
	 * @param xf2
	 */
	public final void collideEdgeAndCircle(final Manifold manifold,
	                                              final EdgeShape edge, final XForm xf1,
	                                              final CircleShape circle, final XForm xf2) {
		manifold.pointCount = 0;

		XForm.mulToOut(xf2, circle.getMemberLocalPosition(), ECc);
		XForm.mulTransToOut(xf1, ECc, ECcLocal);

		final Vec2 n = edge.getNormalVector();
		final Vec2 v1 = edge.getVertex1();
		final Vec2 v2 = edge.getVertex2();
		final float radius = circle.getRadius();
		float separation;

		ECd.set(ECcLocal);
		ECd.subLocal(v1);

		final float dirDist = Vec2.dot(ECd, edge.getDirectionVector());
		if (dirDist <= 0) {

			if (Vec2.dot(ECd, edge.getCorner1Vector()) < 0) {
				return;
			}
			XForm.mulToOut(xf1, v1, ECd);
			ECd.subLocal(ECc);
			ECd.negateLocal();
			// d = c.sub(XForm.mul(xf1, v1));
		} else if (dirDist >= edge.getLength()) {
			ECd.set(ECcLocal);
			ECd.subLocal(v2);
			if (Vec2.dot(ECd, edge.getCorner2Vector()) > 0) {
				return;
			}
			XForm.mulToOut(xf1, v2, ECd);
			ECd.subLocal(ECc);
			ECd.negateLocal();
			//d = c.sub(XForm.mul(xf1, v2));
		} else {
			separation = Vec2.dot(ECd, n);
			if (separation > radius || separation < -radius) {
				return;
			}
			separation -= radius;
			Mat22.mulToOut(xf1.R, n, manifold.normal);
			manifold.pointCount = 1;
			manifold.points[0].id.zero();// key = 0;
			manifold.points[0].separation = separation;
			// just use d as temp vec here, we don't need it any more
			ECd.set(manifold.normal);
			ECd.mulLocal(radius);
			ECc.subLocal(ECd);
			XForm.mulTransToOut(xf1, ECc, manifold.points[0].localPoint1);
			XForm.mulTransToOut(xf2, ECc, manifold.points[0].localPoint2);
			return;
		}

		final float distSqr = Vec2.dot(ECd,ECd);
		if (distSqr > radius * radius) {
			return;
		}

		if (distSqr < Settings.EPSILON) {
			separation = -radius;
			Mat22.mulToOut(xf1.R, n, manifold.normal);
		} else {
			separation = ECd.normalize() - radius;
			manifold.normal.set(ECd);
		}

		manifold.pointCount = 1;
		manifold.points[0].id.zero();//key = 0;
		manifold.points[0].separation = separation;
		// just use d as temp vec here, we don't need it any more
		ECd.set(manifold.normal);
		ECd.mulLocal(radius);
		ECc.subLocal(ECd);
		//c.subLocal(manifold.normal.mul(radius));
		XForm.mulTransToOut(xf1, ECc, manifold.points[0].localPoint1);
		XForm.mulTransToOut(xf2, ECc, manifold.points[0].localPoint2);

	}
}