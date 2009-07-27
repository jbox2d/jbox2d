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

package org.jbox2d.collision;

/*
 * Copyright (c) 2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PointShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

//updated to rev 108->139 of b2cpp

/** Implements the GJK algorithm for computing distance between shapes. */
public final class Distance {
	public int g_GJK_Iterations = 0;

	// These are used to avoid allocations on hot paths:
	private final Vec2 p1s[] = new Vec2[3];
	private final Vec2 p2s[] = new Vec2[3];
	private final Vec2 points[] = new Vec2[3];
	private final Vec2 v = new Vec2();
	private final Vec2 vNeg = new Vec2();
	private final Vec2 w = new Vec2();
	private final Vec2 w1 = new Vec2();
	private final Vec2 w2 = new Vec2();

	public Distance() {
		for (int i = 0; i < 3; ++i) {
			p1s[i] = new Vec2();
			p2s[i] = new Vec2();
			points[i] = new Vec2();
		}
	}

	// GJK using Voronoi regions (Christer Ericson) and region selection
	// optimizations (Casey Muratori).

	// The origin is either in the region of points[1] or in the edge region.
	// The origin is
	// not in region of points[0] because that is the old point.

	// djm pooled
	private final Vec2 p2r = new Vec2();
	private final Vec2 p2d = new Vec2();

	protected final int ProcessTwo(final Vec2 x1, final Vec2 x2, final Vec2[] p1s,
			final Vec2[] p2s, final Vec2[] points) {
		// If in point[1] region
		p2r.x = -points[1].x;
		p2r.y = -points[1].y;
		p2d.x = points[0].x - points[1].x;
		p2d.y = points[0].y - points[1].y;
		final float length = p2d.normalize();
		float lambda = Vec2.dot(p2r, p2d);
		if (lambda <= 0.0f || length < Settings.EPSILON) {
			// The simplex is reduced to a point.
			x1.set(p1s[1]);
			x2.set(p2s[1]);
			p1s[0].set(p1s[1]);
			p2s[0].set(p2s[1]);
			points[0].set(points[1]);
			return 1;
		}

		// Else in edge region
		lambda /= length;
		x1
				.set(p1s[1].x + lambda * (p1s[0].x - p1s[1].x), p1s[1].y + lambda
																* (p1s[0].y - p1s[1].y));
		x2
				.set(p2s[1].x + lambda * (p2s[0].x - p2s[1].x), p2s[1].y + lambda
																* (p2s[0].y - p2s[1].y));
		return 2;
	}

	// Possible regions:
	// - points[2]
	// - edge points[0]-points[2]
	// - edge points[1]-points[2]
	// - inside the triangle
	protected final int ProcessThree(final Vec2 x1, final Vec2 x2, final Vec2[] p1s,
			final Vec2[] p2s, final Vec2[] points) {
		final Vec2 a = points[0];
		final Vec2 b = points[1];
		final Vec2 c = points[2];

		final float abx = b.x - a.x;
		final float aby = b.y - a.y;
		final float acx = c.x - a.x;
		final float acy = c.y - a.y;
		final float bcx = c.x - b.x;
		final float bcy = c.y - b.y;

		final float sn = -(a.x * abx + a.y * aby), sd = b.x * abx + b.y * aby;
		final float tn = -(a.x * acx + a.y * acy), td = c.x * acx + c.y * acy;
		final float un = -(b.x * bcx + b.y * bcy), ud = c.x * bcx + c.y * bcy;

		// In vertex c region?
		if (td <= 0.0f && ud <= 0.0f) {
			// Single point
			x1.set(p1s[2]);
			x2.set(p2s[2]);
			p1s[0].set(p1s[2]);
			p2s[0].set(p2s[2]);
			points[0].set(points[2]);
			return 1;
		}

		// Should not be in vertex a or b region.
		// B2_NOT_USED(sd);
		// B2_NOT_USED(sn);
		assert sn > 0.0f || tn > 0.0f;
		assert sd > 0.0f || un > 0.0f;

		final float n = abx * acy - aby * acx;

		// Should not be in edge ab region.
		final float vc = n * Vec2.cross(a, b);
		assert vc > 0.0f || sn > 0.0f || sd > 0.0f;

		// In edge bc region?
		final float va = n * Vec2.cross(b, c);
		if (va <= 0.0f && un >= 0.0f && ud >= 0.0f && un + ud > 0.0f) {
			assert un + ud > 0.0f;
			final float lambda = un / (un + ud);
			x1.set(p1s[1].x + lambda * (p1s[2].x - p1s[1].x), p1s[1].y + lambda
																* (p1s[2].y - p1s[1].y));
			x2.set(p2s[1].x + lambda * (p2s[2].x - p2s[1].x), p2s[1].y + lambda
																* (p2s[2].y - p2s[1].y));
			p1s[0].set(p1s[2]);
			p2s[0].set(p2s[2]);
			points[0].set(points[2]);
			return 2;
		}

		// In edge ac region?
		final float vb = n * Vec2.cross(c, a);
		if (vb <= 0.0f && tn >= 0.0f && td >= 0.0f && tn + td > 0.0f) {
			assert tn + td > 0.0f;
			final float lambda = tn / (tn + td);
			x1.set(p1s[0].x + lambda * (p1s[2].x - p1s[0].x), p1s[0].y + lambda
																* (p1s[2].y - p1s[0].y));
			x2.set(p2s[0].x + lambda * (p2s[2].x - p2s[0].x), p2s[0].y + lambda
																* (p2s[2].y - p2s[0].y));
			p1s[1].set(p1s[2]);
			p2s[1].set(p2s[2]);
			points[1].set(points[2]);
			return 2;
		}

		// Inside the triangle, compute barycentric coordinates
		float denom = va + vb + vc;
		assert denom > 0.0f;
		denom = 1.0f / denom;
		final float u = va * denom;
		final float v = vb * denom;
		final float w = 1.0f - u - v;
		x1.set(u * p1s[0].x + v * p1s[1].x + w * p1s[2].x, u * p1s[0].y + v * p1s[1].y + w
															* p1s[2].y);
		x2.set(u * p2s[0].x + v * p2s[1].x + w * p2s[2].x, u * p2s[0].y + v * p2s[1].y + w
															* p2s[2].y);
		return 3;
	}

	public final boolean InPoints(final Vec2 w, final Vec2[] points, final int pointCount) {
		final float k_tolerance = 100.0f * Settings.EPSILON;
		for (int i = 0; i < pointCount; ++i) {
			final Vec2 v = points[i];
			// INLINED
			// Vec2 d =Vec2.abs(w.sub(points[i]));
			// new Vec2( MathUtils.abs(w.x-points[i].x),
			// MathUtils.abs(w.y-points[i].y));//Vec2.abs(w - points[i]);
			// Vec2 m = Vec2.max(Vec2.abs(w), Vec2.abs(points[i]));
			final float dx = MathUtils.abs(w.x - v.x);
			final float dy = MathUtils.abs(w.y - v.y);
			final float mx = MathUtils.max(MathUtils.abs(w.x), MathUtils.abs(points[i].x));
			final float my = MathUtils.max(MathUtils.abs(w.y), MathUtils.abs(points[i].y));

			if (dx < k_tolerance * (mx + 1.0f) && dy < k_tolerance * (my + 1.0f)) {
				return true;
			}
		}

		return false;
	}

	/**
	 * Distance between any two objects that implement SupportsGeneric Note that
	 * x1 and x2 are passed so that they may store results - they must be
	 * instantiated before being passed, and the contents will be lost.
	 * 
	 * @param x1
	 *            Set to closest point on shape1 (result parameter)
	 * @param x2
	 *            Set to closest point on shape2 (result parameter)
	 * @param shape1
	 *            Shape to test
	 * @param xf1
	 *            Transform of shape1
	 * @param shape2
	 *            Shape to test
	 * @param xf2
	 *            Transform of shape2
	 * @return the distance
	 */
	// pooled from above
	public final float DistanceGeneric(final Vec2 x1, final Vec2 x2,
			final SupportsGenericDistance shape1, final XForm xf1,
			final SupportsGenericDistance shape2, final XForm xf2) {

		int pointCount = 0;

		shape1.getFirstVertexToOut(xf1, x1);
		shape2.getFirstVertexToOut(xf2, x2);

		float vSqr = 0.0f;
		final int maxIterations = 20;
		for (int iter = 0; iter < maxIterations; ++iter) {
			v.set(x2.x - x1.x, x2.y - x1.y);
			shape1.support(w1, xf1, v);
			vNeg.set(-v.x, -v.y);
			shape2.support(w2, xf2, vNeg);

			vSqr = Vec2.dot(v, v);
			w.set(w2.x - w1.x, w2.y - w1.y);
			final float vw = Vec2.dot(v, w);
			if (vSqr - vw <= 0.01f * vSqr || InPoints(w, points, pointCount)) // or
			// w
			// in
			// points
			{
				if (pointCount == 0) {
					x1.set(w1);
					x2.set(w2);
				}
				g_GJK_Iterations = iter;
				return MathUtils.sqrt(vSqr);
			}

			switch (pointCount) {
				case 0:
					p1s[0].set(w1);
					p2s[0].set(w2);
					points[0].set(w);
					x1.set(p1s[0]);
					x2.set(p2s[0]);
					++pointCount;
					break;

				case 1:
					p1s[1].set(w1);
					p2s[1].set(w2);
					points[1].set(w);
					pointCount = ProcessTwo(x1, x2, p1s, p2s, points);
					break;

				case 2:
					p1s[2].set(w1);
					p2s[2].set(w2);
					points[2].set(w);
					pointCount = ProcessThree(x1, x2, p1s, p2s, points);
					break;
			}

			// If we have three points, then the origin is in the corresponding
			// triangle.
			if (pointCount == 3) {
				g_GJK_Iterations = iter;
				return 0.0f;
				//
			}

			float maxSqr = -Float.MAX_VALUE;// -FLT_MAX;
			for (int i = 0; i < pointCount; ++i) {
				maxSqr = MathUtils.max(maxSqr, Vec2.dot(points[i], points[i]));
			}

			if (pointCount == 3 || vSqr <= 100.0f * Settings.EPSILON * maxSqr) {
				g_GJK_Iterations = iter;
				final float vx = x2.x - x1.x;
				final float vy = x2.y - x1.y;
				vSqr = vx * vx + vy * vy;

				return MathUtils.sqrt(vSqr);
				//
			}
		}

		g_GJK_Iterations = maxIterations;
		return MathUtils.sqrt(vSqr);
		//
	}

	// djm pooled
	private final Vec2 distCCp1 = new Vec2();
	private final Vec2 distCCp2 = new Vec2();
	private final Vec2 distCCd = new Vec2();

	/**
	 * distance between two circle shapes
	 * 
	 * @param x1
	 *            Closest point on shape1 is put here (result parameter)
	 * @param x2
	 *            Closest point on shape2 is put here (result parameter)
	 * @param circle1
	 * @param xf1
	 *            Transform of first shape
	 * @param circle2
	 * @param xf2
	 *            Transform of second shape
	 * @return the distance
	 */
	public final float DistanceCC(final Vec2 x1, final Vec2 x2, final CircleShape circle1,
			final XForm xf1, final CircleShape circle2, final XForm xf2) {

		XForm.mulToOut(xf1, circle1.getMemberLocalPosition(), distCCp1);
		XForm.mulToOut(xf2, circle2.getMemberLocalPosition(), distCCp2);

		distCCd.x = distCCp2.x - distCCp1.x;
		distCCd.y = distCCp2.y - distCCp1.y;
		final float dSqr = Vec2.dot(distCCd, distCCd);
		final float r1 = circle1.getRadius() - Settings.toiSlop;
		final float r2 = circle2.getRadius() - Settings.toiSlop;
		final float r = r1 + r2;
		if (dSqr > r * r) {
			final float dLen = distCCd.normalize();
			final float distance = dLen - r;
			x1.set(distCCp1.x + r1 * distCCd.x, distCCp1.y + r1 * distCCd.y);
			x2.set(distCCp2.x - r2 * distCCd.x, distCCp2.y - r2 * distCCd.y);
			return distance;
		}
		else if (dSqr > Settings.EPSILON * Settings.EPSILON) {
			distCCd.normalize();
			x1.set(distCCp1.x + r1 * distCCd.x, distCCp1.y + r1 * distCCd.y);
			x2.set(x1);
			return 0.0f;
		}

		x1.set(distCCp1);
		x2.set(x1);
		return 0.0f;
	}

	// djm pooled
	private final Vec2 cWorld = new Vec2(); // just like sea world but with less
	// water and more chlorine
	// private Vec2 shamoo = new Vec2();
	private final Vec2 ECcLocal = new Vec2();
	private final Vec2 ECvWorld = new Vec2();
	private final Vec2 ECd = new Vec2();
	private final Vec2 ECtemp = new Vec2();

	/**
	 * Distance bewteen an edge and a circle
	 * 
	 * @param x1
	 *            Closest point on shape1 is put here (result parameter)
	 * @param x2
	 *            Closest point on shape2 is put here (result parameter)
	 * @param edge
	 * @param xf1
	 *            xform of edge
	 * @param circle
	 * @param xf2
	 *            xform of circle
	 * @return the distance
	 */
	public final float DistanceEdgeCircle(final Vec2 x1, final Vec2 x2, final EdgeShape edge,
			final XForm xf1, final CircleShape circle, final XForm xf2) {

		float dSqr;
		float dLen;
		final float r = circle.getRadius() - Settings.toiSlop;
		XForm.mulToOut(xf2, circle.getMemberLocalPosition(), cWorld);
		XForm.mulTransToOut(xf1, cWorld, ECcLocal);
		final float dirDist = Vec2.dot(ECcLocal.sub(edge.getCoreVertex1()), edge
				.getDirectionVector());

		if (dirDist <= 0.0f) {
			XForm.mulToOut(xf1, edge.getCoreVertex1(), ECvWorld);
		}
		else if (dirDist >= edge.getLength()) {
			XForm.mulToOut(xf1, edge.getCoreVertex2(), ECvWorld);
		}
		else {
			x1.set(edge.getDirectionVector());
			x1.mulLocal(dirDist).addLocal(edge.getCoreVertex1());
			XForm.mulToOut(xf1, x1, x1);
			// x1.set(XForm.mul(xf1,
			// edge.getCoreVertex1().add(edge.getDirectionVector().mul(dirDist))));
			ECtemp.set(ECcLocal);
			ECtemp.subLocal(edge.getCoreVertex1());
			dLen = Vec2.dot(ECtemp, edge.getNormalVector());

			if (dLen < 0.0f) {
				if (dLen < -r) {
					x2.set(edge.getNormalVector());
					x2.mulLocal(r).addLocal(ECcLocal);
					XForm.mulToOut(xf1, x2, x2);
					// x2.set(XForm.mul(xf1,
					// ECcLocal.add(edge.getNormalVector().mul(r))));
					return -dLen - r;
				}
				else {
					x2.set(x1);
					return 0.0f;
				}
			}
			else {
				if (dLen > r) {
					x2.set(edge.getNormalVector());
					x2.mulLocal(r).subLocal(ECcLocal).negateLocal();
					XForm.mulToOut(xf1, x2, x2);
					// x2.set(XForm.mul(xf1,
					// ECcLocal.sub(edge.getNormalVector().mul(r))));
					// System.out.println("dlen - r: "+(dLen - r));
					return dLen - r;
				}
				else {
					x2.set(x1);
					return 0.0f;
				}
			}
		}

		x1.set(ECvWorld);
		ECd.set(cWorld);
		ECd.subLocal(ECvWorld);
		dSqr = Vec2.dot(ECd, ECd);
		if (dSqr > r * r) {
			dLen = ECd.normalize();
			x2.set(ECd);
			x2.mulLocal(r).subLocal(cWorld).negateLocal();
			// x2.set(ECcWorld.sub(ECd.mul(r)));
			return dLen - r;
		}
		else {
			x2.set(ECvWorld);
			return 0.0f;
		}
	}

	// GJK is more robust with polygon-vs-point than polygon-vs-circle.
	// So we convert polygon-vs-circle to polygon-vs-point.
	// djm pooled
	private final Point point = new Point();

	/**
	 * Distance between a polygon and a circle
	 * 
	 * @param x1
	 *            Closest point on shape1 is put here (result parameter)
	 * @param x2
	 *            Closest point on shape2 is put here (result parameter)
	 * @param polygon
	 * @param xf1
	 *            xform of polygon
	 * @param circle
	 * @param xf2
	 *            xform of circle
	 * @return the distance
	 */
	public final float DistancePC(final Vec2 x1, final Vec2 x2, final PolygonShape polygon,
			final XForm xf1, final CircleShape circle, final XForm xf2) {
		// v is just used as a dummy Vec2 since it gets overwritten in a moment
		// Point point = new Point(v); djm we don't need this
		// INLINED
		// point.p = XForm.mul(xf2, circle.getLocalPosition());
		point.p.set(xf2.position.x + xf2.R.col1.x * circle.m_localPosition.x + xf2.R.col2.x
					* circle.m_localPosition.y, xf2.position.y + xf2.R.col1.y
												* circle.m_localPosition.x + xf2.R.col2.y
												* circle.m_localPosition.y);

		float distance = DistanceGeneric(x1, x2, polygon, xf1, point, XForm.identity);

		final float r = circle.getRadius() - Settings.toiSlop;

		if (distance > r) {
			distance -= r;
			float dx = x2.x - x1.x;
			float dy = x2.y - x1.y;
			final float length = MathUtils.sqrt(dx * dx + dy * dy);
			if (length >= Settings.EPSILON) {
				final float invLength = 1.0f / length;
				dx *= invLength;
				dy *= invLength;
			}
			x2.x -= r * dx;
			x2.y -= r * dy;
		}
		else {
			distance = 0.0f;
			x2.set(x1);
		}

		return distance;
	}

	/**
	 * Distance between a polygon and a point
	 * 
	 * @param x1
	 *            Closest point on shape1 is put here (result parameter)
	 * @param x2
	 *            Closest point on shape2 is put here (result parameter)
	 * @param polygon
	 * @param xf1
	 *            xform of polygon
	 * @param pt
	 * @param xf2
	 *            xform of point
	 * @return the distance
	 */
	// djm pooled from above
	public final float DistancePolygonPoint(final Vec2 x1, final Vec2 x2,
			final PolygonShape polygon, final XForm xf1, final PointShape pt, final XForm xf2) {
		// v is just used as a dummy Vec2 since it gets overwritten in a moment
		// Point point = new Point(v);
		// INLINED
		// point.p = XForm.mul(xf2, pt.m_localPosition);
		point.p.set(xf2.position.x + xf2.R.col1.x * pt.m_localPosition.x + xf2.R.col2.x
					* pt.m_localPosition.y, xf2.position.y + xf2.R.col1.y * pt.m_localPosition.x
											+ xf2.R.col2.y * pt.m_localPosition.y);

		// TODO: check if we need to subtract toi slop from this...
		float distance = DistanceGeneric(x1, x2, polygon, xf1, point, XForm.identity);
		// ...or if it's better to do it here
		final float r = -Settings.toiSlop;

		if (distance > r) {
			distance -= r;
			float dx = x2.x - x1.x;
			float dy = x2.y - x1.y;
			final float length = MathUtils.sqrt(dx * dx + dy * dy);
			if (length >= Settings.EPSILON) {
				final float invLength = 1.0f / length;
				dx *= invLength;
				dy *= invLength;
			}
			x2.x -= r * dx;
			x2.y -= r * dy;
		}
		else {
			distance = 0.0f;
			x2.set(x1);
		}

		return distance;
	}

	// djm pooled
	private final Vec2 CPp1 = new Vec2();
	private final Vec2 CPp2 = new Vec2();
	private final Vec2 CPd = new Vec2();

	/**
	 * Distance between a circle and a point
	 * 
	 * @param x1
	 *            Closest point on shape1 is put here (result parameter)
	 * @param x2
	 *            Closest point on shape2 is put here (result parameter)
	 * @param circle1
	 * @param xf1
	 *            xform of circle
	 * @param pt2
	 * @param xf2
	 *            xform of point
	 * @return the distance
	 */
	public final float DistanceCirclePoint(final Vec2 x1, final Vec2 x2, final CircleShape circle1,
			final XForm xf1, final PointShape pt2, final XForm xf2) {

		XForm.mulToOut(xf1, circle1.getMemberLocalPosition(), CPp1);
		XForm.mulToOut(xf2, pt2.getMemberLocalPosition(), CPp2);

		CPd.x = CPp2.x - CPp1.x;
		CPd.y = CPp2.y - CPp1.y;
		final float dSqr = Vec2.dot(CPd, CPd);
		final float r1 = circle1.getRadius() - Settings.toiSlop;
		final float r2 = -Settings.toiSlop; // this is necessary, otherwise the
		// toi steps aren't taken
		// correctly...
		final float r = r1 + r2;
		if (dSqr > r * r) {
			final float dLen = CPd.normalize();
			final float distance = dLen - r;
			x1.set(CPp1.x + r1 * CPd.x, CPp1.y + r1 * CPd.y);
			x2.set(CPp2.x - r2 * CPd.x, CPp2.y - r2 * CPd.y);
			return distance;
		}
		else if (dSqr > Settings.EPSILON * Settings.EPSILON) {
			CPd.normalize();
			x1.set(CPp1.x + r1 * CPd.x, CPp1.y + r1 * CPd.y);
			x2.set(x1);
			return 0.0f;
		}

		x1.set(CPp1);
		x2.set(x1);
		return 0.0f;
	}

	/**
	 * Find the closest distance between shapes shape1 and shape2, and load the
	 * closest points into x1 and x2. Note that x1 and x2 are passed so that
	 * they may store results - they must be instantiated before being passed,
	 * and the contents will be lost.
	 * 
	 * @param x1
	 *            Closest point on shape1 is put here (result parameter)
	 * @param x2
	 *            Closest point on shape2 is put here (result parameter)
	 * @param shape1
	 *            First shape to test
	 * @param xf1
	 *            Transform of first shape
	 * @param shape2
	 *            Second shape to test
	 * @param xf2
	 *            Transform of second shape
	 * @return the distance
	 */
	public final float distance(final Vec2 x1, final Vec2 x2, final Shape shape1, final XForm xf1,
			final Shape shape2, final XForm xf2) {

		final ShapeType type1 = shape1.getType();
		final ShapeType type2 = shape2.getType();

		if (type1 == ShapeType.CIRCLE_SHAPE && type2 == ShapeType.CIRCLE_SHAPE) {
			return DistanceCC(x1, x2, (CircleShape) shape1, xf1, (CircleShape) shape2, xf2);
		}
		else if (type1 == ShapeType.POLYGON_SHAPE && type2 == ShapeType.CIRCLE_SHAPE) {
			return DistancePC(x1, x2, (PolygonShape) shape1, xf1, (CircleShape) shape2, xf2);
		}
		else if (type1 == ShapeType.CIRCLE_SHAPE && type2 == ShapeType.POLYGON_SHAPE) {
			return DistancePC(x2, x1, (PolygonShape) shape2, xf2, (CircleShape) shape1, xf1);
		}
		else if (type1 == ShapeType.POLYGON_SHAPE && type2 == ShapeType.POLYGON_SHAPE) {
			return DistanceGeneric(x1, x2, (PolygonShape) shape1, xf1, (PolygonShape) shape2, xf2);
		}
		else if (type1 == ShapeType.EDGE_SHAPE && type2 == ShapeType.CIRCLE_SHAPE) {
			return DistanceEdgeCircle(x1, x2, (EdgeShape) shape1, xf1, (CircleShape) shape2, xf2);
		}
		else if (type1 == ShapeType.CIRCLE_SHAPE && type2 == ShapeType.EDGE_SHAPE) {
			return DistanceEdgeCircle(x2, x1, (EdgeShape) shape2, xf2, (CircleShape) shape1, xf1);
		}
		else if (type1 == ShapeType.POLYGON_SHAPE && type2 == ShapeType.EDGE_SHAPE) {
			return DistanceGeneric(x2, x1, (EdgeShape) shape2, xf2, (PolygonShape) shape1, xf1);
		}
		else if (type1 == ShapeType.EDGE_SHAPE && type2 == ShapeType.POLYGON_SHAPE) {
			return DistanceGeneric(x1, x2, (EdgeShape) shape1, xf1, (PolygonShape) shape2, xf2);
		}
		else if (type1 == ShapeType.POINT_SHAPE && type2 == ShapeType.POINT_SHAPE) {
			return Float.MAX_VALUE;
		}
		else if (type1 == ShapeType.POINT_SHAPE && type2 == ShapeType.CIRCLE_SHAPE) {
			return DistanceCirclePoint(x2, x1, (CircleShape) shape2, xf2, (PointShape) shape1, xf1);
		}
		else if (type1 == ShapeType.CIRCLE_SHAPE && type2 == ShapeType.POINT_SHAPE) {
			return DistanceCirclePoint(x1, x2, (CircleShape) shape1, xf1, (PointShape) shape2, xf2);
		}
		else if (type1 == ShapeType.POINT_SHAPE && type2 == ShapeType.POLYGON_SHAPE) {
			return DistancePolygonPoint(x2, x1, (PolygonShape) shape2, xf2, (PointShape) shape1,
					xf1);
		}
		else if (type1 == ShapeType.POLYGON_SHAPE && type2 == ShapeType.POINT_SHAPE) {
			return DistancePolygonPoint(x1, x2, (PolygonShape) shape1, xf1, (PointShape) shape2,
					xf2);
		}

		return 0.0f;
	}

}

// This is used for polygon-vs-circle distance.
class Point implements SupportsGenericDistance {
	public Vec2 p;

	public Point(final Vec2 _p) {
		p = _p.clone();
	}

	public Point() {
		p = new Vec2();
	}

	public void support(final Vec2 dest, final XForm xf, final Vec2 v) {
		dest.set(p);
	}

	public void getFirstVertexToOut(final XForm xf, final Vec2 out) {
		out.set(p);
	}
}