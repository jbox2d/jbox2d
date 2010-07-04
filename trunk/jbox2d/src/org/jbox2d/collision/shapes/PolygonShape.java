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

import java.util.List;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.MassData;
import org.jbox2d.collision.OBB;
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.SegmentCollide;
import org.jbox2d.collision.SupportsGenericDistance;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.RaycastResult;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.pooling.TLAABB;
import org.jbox2d.pooling.TLMassData;
import org.jbox2d.pooling.TLMat22;
import org.jbox2d.pooling.TLVec2;


//Updated to rev 142 of b2Shape.cpp/.h / b2PolygonShape.cpp/.h

/** A convex polygon shape.  Create using Body.createShape(ShapeDef), not the constructor here. */
public class PolygonShape extends Shape implements SupportsGenericDistance{
	/** Dump lots of debug information. */
	private static boolean m_debug = false;

	/** Local position of the shape centroid in parent body frame. */
	public final Vec2 m_centroid;

	/** The oriented bounding box of the shape. */
	public final OBB m_obb;

	/** The vertices of the shape.  Note: use getVertexCount(), not m_vertices.length, to get number of active vertices. */
	public final Vec2 m_vertices[];
	/** The normals of the shape.  Note: use getVertexCount(), not m_normals.length, to get number of active normals. */
	public final Vec2 m_normals[];
	/** The normals of the shape.  Note: use getVertexCount(), not m_coreVertices.length, to get number of active vertices. */
	public final Vec2 m_coreVertices[];
	/** Number of active vertices in the shape. */
	public int m_vertexCount;

	// djm pooling
	private static final TLVec2 tlEdge = new TLVec2();
	private static final TLVec2 tlV = new TLVec2();
	private static final TLVec2 tlD = new TLVec2();
	private static final TLMat22 tlA = new TLMat22();
	
	public PolygonShape(final ShapeDef def) {
		super(def);

		assert(def.type == ShapeType.POLYGON_SHAPE);
		m_type = ShapeType.POLYGON_SHAPE;
		final PolygonDef poly = (PolygonDef)def;

		m_vertexCount = poly.getVertexCount();
		m_vertices = new Vec2[m_vertexCount];
		m_normals = new Vec2[m_vertexCount];
		m_coreVertices = new Vec2[m_vertexCount];

		m_obb = new OBB();

		// Get the vertices transformed into the body frame.
		assert(3 <= m_vertexCount && m_vertexCount <= Settings.maxPolygonVertices);

		// Copy vertices.
		for (int i = 0; i < m_vertexCount; ++i) {
			m_vertices[i] = poly.vertices.get(i).clone();
		}

		// Compute normals. Ensure the edges have non-zero length.
		final Vec2 edge = tlEdge.get();
		for (int i = 0; i < m_vertexCount; ++i) {
			final int i1 = i;
			final int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
			edge.set(m_vertices[i2]).subLocal(m_vertices[i1]);
			assert(edge.lengthSquared() > Settings.EPSILON*Settings.EPSILON);
			m_normals[i] = Vec2.cross(edge, 1.0f);
			m_normals[i].normalize();
		}

		if (m_debug) {
			// Ensure the polygon is convex.
			for (int i = 0; i < m_vertexCount; ++i) {
				for (int j = 0; j < m_vertexCount; ++j) {
					// Don't check vertices on the current edge.
					if (j == i || j == (i + 1) % m_vertexCount)
					{
						continue;
					}

					// Your polygon is non-convex (it has an indentation).
					// Or your polygon is too skinny.

					// djm: don't worry about the extra allocations here.
					// also, don't calculate this unless we need assertions
					//float s = Vec2.dot(m_normals[i], m_vertices[j].sub(m_vertices[i]));
					//assert(s < -Settings.linearSlop);
					assert( Vec2.dot(m_normals[i], m_vertices[j].sub(m_vertices[i])) < -Settings.linearSlop);
				}
			}

			// Ensure the polygon is counter-clockwise.
			for (int i = 1; i < m_vertexCount; ++i) {
				float cross = Vec2.cross(m_normals[i-1], m_normals[i]);

				// Keep asinf happy.
				cross = MathUtils.clamp(cross, -1.0f, 1.0f);

				// You have consecutive edges that are almost parallel on your polygon.
				// Or the polygon is not counter-clockwise.
				final float angle = (float)Math.asin(cross);
				assert(angle > Settings.angularSlop);
			}
			//#endif
		}

		// Compute the polygon centroid.
		m_centroid = PolygonShape.computeCentroid(poly.vertices);

		// Compute the oriented bounding box.
		PolygonShape.computeOBB(m_obb, m_vertices);

		
		final Vec2 v = tlV.get();
		final Vec2 d = tlD.get();
		final Mat22 A = tlA.get();
		
		// Create core polygon shape by shifting edges inward.
		// Also compute the min/max radius for CCD.
		for (int i = 0; i < m_vertexCount; ++i) {
			final int i1 = i - 1 >= 0 ? i - 1 : m_vertexCount - 1;
			final int i2 = i;

			final Vec2 n1 = m_normals[i1];
			final Vec2 n2 = m_normals[i2];
			v.set(m_vertices[i]).subLocal(m_centroid);

			d.x = Vec2.dot(n1, v) - Settings.toiSlop;
			d.y = Vec2.dot(n2, v) - Settings.toiSlop;

			// Shifting the edge inward by b2_toiSlop should
			// not cause the plane to pass the centroid.

			// Your shape has a radius/extent less than b2_toiSlop.
			if ((d.x < 0.0f || d.y < 0.0f)) {
				System.out.println("Error, polygon extents less than b2_toiSlop, dumping details: ");
				System.out.println("d.x: "+d.x+"d.y: "+d.y);
				System.out.println("n1: "+n1+"; n2: "+n2);
				System.out.println("v: "+v);
			}
			assert(d.x >= 0.0f);
			assert(d.y >= 0.0f);
			A.col1.x = n1.x; A.col2.x = n1.y;
			A.col1.y = n2.x; A.col2.y = n2.y;
			m_coreVertices[i] = A.solve(d).addLocal(m_centroid);
		}

		if (m_debug) {
			System.out.println("\nDumping polygon shape...");
			System.out.println("Vertices: ");
			for (int i=0; i<m_vertexCount; ++i) {
				System.out.println(m_vertices[i]);
			}
			System.out.println("Core Vertices: ");
			for (int i=0; i<m_vertexCount; ++i) {
				System.out.println(m_coreVertices[i]);
			}
			System.out.println("Normals: ");
			for (int i=0; i<m_vertexCount; ++i) {
				System.out.println(m_normals[i]);
			}
			System.out.println("Centroid: "+m_centroid);

		}
	}

	// djm pooling, from above
	/**
	 * @see Shape#updateSweepRadius(Vec2)
	 */
	@Override
	public void updateSweepRadius(final Vec2 center) {
		// Update the sweep radius (maximum radius) as measured from
		// a local center point.
		final Vec2 d = tlD.get();
		m_sweepRadius = 0.0f;
		for (int i = 0; i < m_vertexCount; ++i) {
			d.set(m_coreVertices[i]);
			d.subLocal(center);
			m_sweepRadius = MathUtils.max(m_sweepRadius, d.length());
		}
	}

	// djm pooling
	private static final TLVec2 tlTemp = new TLVec2();
	private static final TLVec2 tlPLocal= new TLVec2();
	/**
	 * @see Shape#testPoint(XForm, Vec2)
	 */
	@Override
	public boolean testPoint(final XForm xf, final Vec2 p) {
		final Vec2 temp = tlTemp.get();
		final Vec2 pLocal = tlPLocal.get();
		
		temp.set(p);
		temp.subLocal(xf.position);
		Mat22.mulTransToOut(xf.R, temp, pLocal);

		if (m_debug) {
			System.out.println("--testPoint debug--");
			System.out.println("Vertices: ");
			for (int i=0; i < m_vertexCount; ++i) {
				System.out.println(m_vertices[i]);
			}
			System.out.println("pLocal: "+pLocal);
		}

		for (int i = 0; i < m_vertexCount; ++i) {
			temp.set(pLocal);
			temp.subLocal( m_vertices[i]);
			final float dot = Vec2.dot(m_normals[i], temp);

			if (dot > 0.0f) {
				return false;
			}
		}

		return true;
	}


	// djm pooling, and from above
	private static final TLVec2 tlP1 = new TLVec2();
	private static final TLVec2 tlP2 = new TLVec2();
	/**
	 * @see Shape#testSegment(XForm, RaycastResult, Segment, float)
	 */
	@Override
	public SegmentCollide testSegment(final XForm xf, final RaycastResult out, final Segment segment, final float maxLambda){

		float lower = 0.0f, upper = maxLambda;
		
		final Vec2 p1 = tlP1.get();
		final Vec2 p2 = tlP2.get();
		final Vec2 d = tlD.get();
		final Vec2 temp = tlTemp.get();
		
		/*Vec2 p1 = Mat22.mulTrans(xf.R, segment.p1.sub(xf.position));
		Vec2 p2 = Mat22.mulTrans(xf.R, segment.p2.sub(xf.position));
		Vec2 d = p2.sub(p1);*/
		
		p1.set(segment.p1).subLocal(xf.position);
		Mat22.mulTransToOut(xf.R, p1, p1);
		p2.set(segment.p2).subLocal(xf.position);
		Mat22.mulTransToOut(xf.R, p2, p2);
		d.set(p2).subLocal(p1);

		int index = -1;

		for (int i = 0; i < m_vertexCount; ++i){
			// p = p1 + a * d
			// dot(normal, p - v) = 0
			// dot(normal, p1 - v) + a * dot(normal, d) = 0
//			float numerator = Vec2.dot(m_normals[i],m_vertices[i].sub(p1));
//			float denominator = Vec2.dot(m_normals[i],d);
			
			temp.set(m_vertices[i]).subLocal(p1);
			final float numerator = Vec2.dot(m_normals[i], temp);
			final float denominator = Vec2.dot(m_normals[i], d);

			if (denominator == 0.0f){	
				
				if (numerator < 0.0f){
					return SegmentCollide.MISS_COLLIDE;
				}
			}
			
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.

			if (denominator < 0.0f && numerator < lower * denominator){
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i;
			}
			else if (denominator > 0.0f && numerator < upper * denominator){
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}

			if (upper < lower){
				return SegmentCollide.MISS_COLLIDE;
			}
		}

		assert(0.0f <= lower && lower <= maxLambda);

		if (index >= 0){
			out.lambda = lower;
			Mat22.mulToOut(xf.R, m_normals[index], out.normal);
			//out.normal.set(Mat22.mul(xf.R,m_normals[index]));
			return SegmentCollide.HIT_COLLIDE;
		}
		
		out.lambda = 0.0f;
		return SegmentCollide.STARTS_INSIDE_COLLIDE;
	}

	// djm pooling
	private static final TLVec2 tlSupDLocal = new TLVec2();
	/**
	 * Get the support point in the given world direction.
	 * Use the supplied transform.
	 * @see SupportsGenericDistance#support(Vec2, XForm, Vec2)
	 */
	public void support(final Vec2 dest, final XForm xf, final Vec2 d) {
		final Vec2 supportDLocal = tlSupDLocal.get();
		Mat22.mulTransToOut(xf.R, d, supportDLocal);

		int bestIndex = 0;
		float bestValue = Vec2.dot(m_coreVertices[0], supportDLocal);
		for (int i = 1; i < m_vertexCount; ++i) {
			final float value = Vec2.dot(m_coreVertices[i], supportDLocal);
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}

		XForm.mulToOut(xf, m_coreVertices[bestIndex], dest);
	}
	
	// djm pooling, and from above
	private static final TLVec2 tlPRef = new TLVec2();
	private static final TLVec2 tlE1 = new TLVec2();
	private static final TLVec2 tlE2 = new TLVec2();
	public final static Vec2 computeCentroid(final List<Vec2> vs) {
		final int count = vs.size();
		assert(count >= 3);

		final Vec2 c = new Vec2();
		float area = 0.0f;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		final Vec2 pRef = tlPRef.get();
		pRef.setZero();
		//    #if 0
		//        // This code would put the reference point inside the polygon.
		//        for (int32 i = 0; i < count; ++i)
		//        {
		//            pRef += vs[i];
		//        }
		//        pRef *= 1.0f / count;
		//    #endif

		final float inv3 = 1.0f / 3.0f;

		final Vec2 e1 = tlE1.get();
		final Vec2 e2 = tlE2.get();
		final Vec2 p1 = tlP1.get();
		
		for (int i = 0; i < count; ++i) {
			// Triangle vertices.
			p1.set(pRef);
			final Vec2 p2 = vs.get(i);
			final Vec2 p3 = i + 1 < count ? vs.get(i+1) : vs.get(0);

			e1.set(p2).subLocal(p1);
			e2.set(p3).subLocal(p1);

			final float D = Vec2.cross(e1, e2);

			final float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid
			//c += triangleArea * inv3 * (p1 + p2 + p3);
			c.x += triangleArea * inv3 * (p1.x + p2.x + p3.x);
			c.y += triangleArea * inv3 * (p1.y + p2.y + p3.y);

		}

		// Centroid
		assert(area > Settings.EPSILON);
		c.mulLocal(1.0f / area);
		return c;
	}

	// djm pooling, and from above
	private static final TLVec2 tlUX = new TLVec2();
	private static final TLVec2 tlUY = new TLVec2();
	private static final TLVec2 tlLower = new TLVec2();
	private static final TLVec2 tlUpper = new TLVec2();
	private static final TLVec2 tlR = new TLVec2();
	private static final TLVec2 tlCenter = new TLVec2();
	
	// http://www.geometrictools.com/Documentation/MinimumAreaRectangle.pdf
	public static void computeOBB(final OBB obb, final Vec2[] vs){
		final int count = vs.length;
		assert(count <= Settings.maxPolygonVertices);
		
		final Vec2 ux = tlUX.get();
		final Vec2 uy = tlUY.get();
		final Vec2 lower = tlLower.get();
		final Vec2 upper = tlUpper.get();
		final Vec2 d = tlD.get();
		final Vec2 r = tlR.get();
		final Vec2 center = tlCenter.get();
		
		final Vec2[] pRay = new Vec2[Settings.maxPolygonVertices + 1];
		for (int i = 0; i < count; ++i){
			pRay[i] = vs[i];
		}
		pRay[count] = pRay[0];

		float minArea = Float.MAX_VALUE;
		
		for (int i = 1; i <= count; ++i){
			final Vec2 root = pRay[i-1];
			ux.set(pRay[i]);
			ux.subLocal(root);
			final float length = ux.normalize();
			assert(length > Settings.EPSILON);
			uy.x = -ux.y;
			uy.y = ux.x;
			lower.x = Float.MAX_VALUE;
			lower.y = Float.MAX_VALUE;
			upper.x = -Float.MAX_VALUE; // djm wouldn't this just be Float.MIN_VALUE?
			upper.y = -Float.MAX_VALUE;

			for (int j = 0; j < count; ++j) {
				d.set(pRay[j]);
				d.subLocal(root);
				r.x = Vec2.dot(ux, d);
				r.y = Vec2.dot(uy, d);
				Vec2.minToOut(lower, r, lower);
				Vec2.maxToOut(upper, r, upper);
			}

			final float area = (upper.x - lower.x) * (upper.y - lower.y);
			if (area < 0.95f * minArea){
				minArea = area;
				obb.R.col1.set(ux);
				obb.R.col2.set(uy);

				center.set(0.5f * (lower.x + upper.x), 0.5f * (lower.y + upper.y));
				Mat22.mulToOut(obb.R, center, obb.center);
				obb.center.addLocal(root);
				//obb.center = root.add(Mat22.mul(obb.R, center));

				obb.extents.x = 0.5f * (upper.x - lower.x);
				obb.extents.y = 0.5f * (upper.y - lower.y);
			}
		}

		assert(minArea < Float.MAX_VALUE);
	}

	private static final TLMat22 tlCaabbR = new TLMat22();
	private static final TLVec2 tlCaabbH = new TLVec2();
	/**
	 * @see Shape#computeAABB(AABB, XForm)
	 */
	@Override
	public void computeAABB(final AABB aabb, final XForm xf) {
		/*Mat22 R = Mat22.mul(xf.R, m_obb.R);
		Mat22 absR = Mat22.abs(R);
		Vec2 h = Mat22.mul(absR, m_obb.extents);
		Vec2 position = xf.position.add(Mat22.mul(xf.R, m_obb.center));*/
		final Mat22 caabbR = tlCaabbR.get();
		final Vec2 caabbH = tlCaabbH.get();

		Mat22.mulToOut(xf.R, m_obb.R, caabbR);
		caabbR.absLocal();
		Mat22.mulToOut(caabbR, m_obb.extents, caabbH);
		// we treat the lowerbound like the position
		Mat22.mulToOut(xf.R, m_obb.center, aabb.lowerBound);
		aabb.lowerBound.addLocal(xf.position);

		aabb.upperBound.set(aabb.lowerBound);

		aabb.lowerBound.subLocal(caabbH);
		aabb.upperBound.addLocal(caabbH);
	}

	// djm pooling, hot method
	private static final TLAABB tlSwept1 = new TLAABB();
	private static final TLAABB tlSwept2 = new TLAABB();
	/**
	 * @see Shape#computeSweptAABB(AABB, XForm, XForm)
	 */
	@Override
	public void computeSweptAABB(final AABB aabb, final XForm transform1, final XForm transform2) {
		
		final AABB sweptAABB1 = tlSwept1.get();
		final AABB sweptAABB2 = tlSwept2.get();
		
		computeAABB(sweptAABB1, transform1);
		computeAABB(sweptAABB2, transform2);
		Vec2.minToOut(sweptAABB1.lowerBound, sweptAABB2.lowerBound, aabb.lowerBound);
		Vec2.maxToOut(sweptAABB1.upperBound, sweptAABB2.upperBound, aabb.upperBound);
		//System.out.println("poly sweepaabb: "+aabb.lowerBound+" "+aabb.upperBound);
	}

	@Override
	public void computeMass(final MassData massData) {
		computeMass(massData, m_density);
	}
	
	//djm pooling, from above
	/**
	 * @see Shape#computeMass(MassData)
	 */
	public void computeMass(final MassData massData, float density) {
		// Polygon mass, centroid, and inertia.
		// Let rho be the polygon density in mass per unit area.
		// Then:
		// mass = rho * int(dA)
		// centroid.x = (1/mass) * rho * int(x * dA)
		// centroid.y = (1/mass) * rho * int(y * dA)
		// I = rho * int((x*x + y*y) * dA)
		//
		// We can compute these integrals by summing all the integrals
		// for each triangle of the polygon. To evaluate the integral
		// for a single triangle, we make a change of variables to
		// the (u,v) coordinates of the triangle:
		// x = x0 + e1x * u + e2x * v
		// y = y0 + e1y * u + e2y * v
		// where 0 <= u && 0 <= v && u + v <= 1.
		//
		// We integrate u from [0,1-v] and then v from [0,1].
		// We also need to use the Jacobian of the transformation:
		// D = cross(e1, e2)
		//
		// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
		//
		// The rest of the derivation is handled by computer algebra.

		assert(m_vertexCount >= 3);

		final Vec2 center = tlCenter.get();
		center.setZero();
		float area = 0.0f;
		float I = 0.0f;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		final Vec2 pRef = tlPRef.get();
		pRef.setZero();

		final float k_inv3 = 1.0f / 3.0f;

		final Vec2 e1 = tlE1.get();
		final Vec2 e2 = tlE2.get();

		for (int i = 0; i < m_vertexCount; ++i) {
			// Triangle vertices.
			final Vec2 p1 = pRef;
			final Vec2 p2 = m_vertices[i];
			final Vec2 p3 = i + 1 < m_vertexCount ? m_vertices[i+1] : m_vertices[0];

			e1.set(p2);
			e1.subLocal(p1);

			e2.set(p3);
			e2.subLocal(p1);

			final float D = Vec2.cross(e1, e2);

			final float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid
			center.x += triangleArea * k_inv3 * (p1.x + p2.x + p3.x);
			center.y += triangleArea * k_inv3 * (p1.y + p2.y + p3.y);

			final float px = p1.x, py = p1.y;
			final float ex1 = e1.x, ey1 = e1.y;
			final float ex2 = e2.x, ey2 = e2.y;

			final float intx2 = k_inv3 * (0.25f * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5f*px*px;
			final float inty2 = k_inv3 * (0.25f * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5f*py*py;

			I += D * (intx2 + inty2);
		}

		// Total mass
		massData.mass = density * area;

		// Center of mass
		assert(area > Settings.EPSILON);
		center.mulLocal(1.0f / area);
		massData.center.set(center);

		// Inertia tensor relative to the local origin.
		massData.I = I*density;
	}

	/** Get the first vertex and apply the supplied transform. */
	public void getFirstVertexToOut(final XForm xf, final Vec2 out) {
		XForm.mulToOut(xf, m_coreVertices[0], out);
	}

	/** Get the oriented bounding box relative to the parent body. */
	public OBB getOBB() {
		return m_obb.clone();
	}

	/** Get the local centroid relative to the parent body. */
	public Vec2 getCentroid() {
		return m_centroid.clone();
	}

	/** Get the number of vertices. */
	public int getVertexCount() {
		return m_vertexCount;
	}

	/** Get the vertices in local coordinates. */
	public Vec2[] getVertices() {
		return m_vertices;
	}

	/**
	 * Get the core vertices in local coordinates. These vertices
	 * represent a smaller polygon that is used for time of impact
	 * computations.
	 */
	public Vec2[] getCoreVertices()	{
		return m_coreVertices;
	}

	/** Get the edge normal vectors.  There is one for each vertex. */
	public Vec2[] getNormals() {
		return m_normals;
	}

	/** Get the centroid and apply the supplied transform. */
	public Vec2 centroid(final XForm xf) {
		return XForm.mul(xf, m_centroid);
	}
	
	// djm pooling, and from above
	private static final TLVec2 tlNormalL = new TLVec2();
	private static final TLMassData tlMd = new TLMassData();
	private static final TLVec2 tlIntoVec = new TLVec2();
	private static final TLVec2 tlOutoVec = new TLVec2();
	private static final TLVec2 tlP2b = new TLVec2();
	private static final TLVec2 tlP3 = new TLVec2();
	/**
	 * @see Shape#computeSubmergedArea(Vec2, float, XForm, Vec2)
	 */
	public float computeSubmergedArea(final Vec2 normal, float offset, XForm xf, Vec2 c) {
		final Vec2 normalL = tlNormalL.get();
		final MassData md = tlMd.get();
		
		//Transform plane into shape co-ordinates
		Mat22.mulTransToOut(xf.R,normal, normalL);
		float offsetL = offset - Vec2.dot(normal,xf.position);
		
		float[] depths = new float[Settings.maxPolygonVertices];
		int diveCount = 0;
		int intoIndex = -1;
		int outoIndex = -1;
		
		boolean lastSubmerged = false;
		int i = 0;
		for (i = 0; i < m_vertexCount; ++i){
			depths[i] = Vec2.dot(normalL,m_vertices[i]) - offsetL;
			boolean isSubmerged = depths[i]<-Settings.EPSILON;
			if (i > 0){
				if (isSubmerged){
					if (!lastSubmerged){
						intoIndex = i-1;
						diveCount++;
					}
				}
				else{
					if (lastSubmerged){
						outoIndex = i-1;
						diveCount++;
					}
				}
			}
			lastSubmerged = isSubmerged;
		}

		switch(diveCount){
		case 0:
			if (lastSubmerged){
				//Completely submerged
				computeMass(md, 1.0f);
				XForm.mulToOut(xf,md.center, c);
				return md.mass;
			}
			else{
				return 0;
			}

		case 1:
			if(intoIndex==-1){
				intoIndex = m_vertexCount-1;
			}
			else{
				outoIndex = m_vertexCount-1;
			}
			break;
		}

		final Vec2 intoVec = tlIntoVec.get();
		final Vec2 outoVec = tlOutoVec.get();
		final Vec2 e1 = tlE1.get();
		final Vec2 e2 = tlE2.get();
		
		int intoIndex2 = (intoIndex+1) % m_vertexCount;
		int outoIndex2 = (outoIndex+1) % m_vertexCount;
		
		float intoLambda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
		float outoLambda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
		
		intoVec.set(m_vertices[intoIndex].x*(1-intoLambda)+m_vertices[intoIndex2].x*intoLambda,
						m_vertices[intoIndex].y*(1-intoLambda)+m_vertices[intoIndex2].y*intoLambda);
		outoVec.set(m_vertices[outoIndex].x*(1-outoLambda)+m_vertices[outoIndex2].x*outoLambda,
						m_vertices[outoIndex].y*(1-outoLambda)+m_vertices[outoIndex2].y*outoLambda);
		
		// Initialize accumulator
		float area = 0;
		final Vec2 center = tlCenter.get();
		center.setZero();
		final Vec2 p2b = tlP2b.get().set(m_vertices[intoIndex2]);
		final Vec2 p3 = tlP3.get();
		p3.setZero();
		
		float k_inv3 = 1.0f / 3.0f;
		
		// An awkward loop from intoIndex2+1 to outIndex2
		i = intoIndex2;
		while (i != outoIndex2){
			i = (i+1) % m_vertexCount;
			if (i == outoIndex2){
				p3.set(outoVec);				
			}
			else{
				p3.set(m_vertices[i]);				
			}
			
			// Add the triangle formed by intoVec,p2,p3
			{
				e1.set(p2b).subLocal(intoVec);
				e2.set(p3).subLocal(intoVec);
				
				float D = Vec2.cross(e1, e2);
				
				float triangleArea = 0.5f * D;

				area += triangleArea;
				
				// Area weighted centroid
				center.x += triangleArea * k_inv3 * (intoVec.x + p2b.x + p3.x);
				center.y += triangleArea * k_inv3 * (intoVec.y + p2b.y + p3.y);
			}
			//
			p2b.set(p3);
		}
		
		// Normalize and transform centroid
		center.x *= 1.0f / area;
		center.y *= 1.0f / area;
		
		XForm.mulToOut(xf, center, c);
		
		return area;
	}
}