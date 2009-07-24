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

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.MassData;
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.structs.SegmentCollide;
import org.jbox2d.collision.structs.ShapeType;
import org.jbox2d.collision.structs.TestSegmentResult;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;


//Updated to rev 142 of Shape.cpp/.h / PolygonShape.cpp/.h

/** A convex polygon shape.  Create using Body.createShape(ShapeDef), not the ructor here. */
public class PolygonShape extends Shape{
	/** Dump lots of debug information. */
	private static boolean m_debug = false; 

	/**
	 * Local position of the shape centroid in parent body frame.
	 */
	public final Vec2 m_centroid = new Vec2();

	/**
	 * The vertices of the shape.  Note: use getVertexCount(), not m_vertices.length, to get number of active vertices.
	 */
	public final Vec2 m_vertices[];

	/**
	 * The normals of the shape.  Note: use getVertexCount(), not m_normals.length, to get number of active normals.
	 */
	public final Vec2 m_normals[];

	/**
	 * Number of active vertices in the shape.
	 */
	public int m_vertexCount;

	public PolygonShape() {
		m_type = ShapeType.POLYGON_SHAPE;

		m_vertexCount = 0;
		m_vertices = new Vec2[Settings.maxPolygonVertices];
		m_normals = new Vec2[Settings.maxPolygonVertices];
		m_radius = Settings.polygonRadius;
	}

	// djm pooled
	private static final Vec2 edge = new Vec2();
	private static final Vec2 r = new Vec2();
	/**
	 * Copy vertices. This assumes the vertices define a convex polygon.
	 * It is assumed that the exterior is the the right of each edge.
	 */
	public final void set( final Vec2[] vertices, final int count){
		assert(3 <= count && count <= Settings.maxPolygonVertices);
		m_vertexCount = count;

		// Copy vertices.
		for (int i = 0; i < m_vertexCount; ++i){
			m_vertices[i] = vertices[i];
		}

		// Compute normals. Ensure the edges have non-zero length.
		for (int i = 0; i < m_vertexCount; ++i){
			final int i1 = i;
			final int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
			edge.set(m_vertices[i2]).subLocal(m_vertices[i1]);
			assert(edge.lengthSquared() > Settings.EPSILON * Settings.EPSILON);
			Vec2.crossToOut( edge, 1f, m_normals[i]);
			m_normals[i].normalize();
		}

		if(m_debug){

			// Ensure the polygon is convex and the interior
			// is to the left of each edge.
			for (int i = 0; i < m_vertexCount; ++i){
				final int i1 = i;
				final int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
				edge.set(m_vertices[i2]).subLocal(m_vertices[i1]);

				for (int j = 0; j < m_vertexCount; ++j){
					// Don't check vertices on the current edge.
					if (j == i1 || j == i2){
						continue;
					}

					r.set(m_vertices[j]).subLocal(m_vertices[i1]);

					// Your polygon is non-convex (it has an indentation) or
					// has colinear edges.
					final float s = Vec2.cross(edge, r);
					assert(s > 0.0f);
				}
			}
		}

		// Compute the polygon centroid.
		computeCentroidToOut(m_vertices, m_vertexCount, m_centroid);
	}

	/**
	 * Build vertices to represent an axis-aligned box.
	 * @param hx the half-width.
	 * @param hy the half-height.
	 */
	public final void setAsBox(final float hx, final float hy){
		m_vertexCount = 4;
		m_vertices[0].set(-hx, -hy);
		m_vertices[1].set( hx, -hy);
		m_vertices[2].set( hx,  hy);
		m_vertices[3].set(-hx,  hy);
		m_normals[0].set(0.0f, -1.0f);
		m_normals[1].set(1.0f, 0.0f);
		m_normals[2].set(0.0f, 1.0f);
		m_normals[3].set(-1.0f, 0.0f);
		m_centroid.setZero();
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
	public final void setAsBox(final float hx, final float hy, final Vec2 center, final float angle){
		m_vertexCount = 4;
		m_vertices[0].set(-hx, -hy);
		m_vertices[1].set( hx, -hy);
		m_vertices[2].set( hx,  hy);
		m_vertices[3].set(-hx,  hy);
		m_normals[0].set(0.0f, -1.0f);
		m_normals[1].set(1.0f, 0.0f);
		m_normals[2].set(0.0f, 1.0f);
		m_normals[3].set(-1.0f, 0.0f);
		m_centroid.set(center);

		xf.position.set(center);
		xf.R.set(angle);

		// Transform vertices and normals.
		for (int i = 0; i < m_vertexCount; ++i){
			XForm.mulToOut( xf, m_vertices[i], m_vertices[i]);
			Mat22.mulToOut( xf.R, m_normals[i], m_normals[i]);
		}
	}

	/**
	 * Set this as a single edge.
	 * @param v1
	 * @param v2
	 */
	public final void setAsEdge( final Vec2 v1,  final Vec2 v2){
		m_vertexCount = 2;
		m_vertices[0].set(v1);
		m_vertices[1].set(v2);
		m_centroid.set(v1).addLocal(v2).mulLocal( 0.5f);
		//= 0.5f * (v1 + v2);
		m_normals[0].set(v2).subLocal( v1);
		Vec2.crossToOut( m_normals[0], 1f, m_normals[0]);
		//m_normals[0] = Cross(v2 - v1, 1.0f);
		m_normals[0].normalize();
		m_normals[1].set(m_normals[0]).negateLocal();
	}

	// djm pooling
	private static final Vec2 pLocal = new Vec2();
	private static final Vec2 temp = new Vec2();
	/**
	 * @see Shape#testPoint(XForm, Vec2)
	 */
	@Override
	public final boolean testPoint( final XForm transform,  final Vec2 p){
		pLocal.set(p).subLocal(xf.position);
		Mat22.mulTransToOut( xf.R, pLocal, pLocal);

		if (m_debug) {
			System.out.println("--testPoint debug--");
			System.out.println("Vertices: ");
			for (int i=0; i < m_vertexCount; ++i) {
				System.out.println(m_vertices[i]);
			}
			System.out.println("pLocal: "+pLocal);
		}


		for (int i = 0; i < m_vertexCount; ++i){
			temp.set(pLocal);
			temp.subLocal( m_vertices[i]);
			final float dot = Vec2.dot(m_normals[i], temp);
			if (dot > 0.0f){
				return false;
			}
		}

		return true;
	}

	// djm pooling
	private static final Vec2 lower = new Vec2();
	private static final Vec2 upper = new Vec2();
	private static final Vec2 v = new Vec2();
	/**
	 * @see Shape#computeAABB(AABB, XForm)
	 */
	@Override
	public final void computeAABB(final AABB aabb,  final XForm xf){
		XForm.mulToOut(xf, m_vertices[0], lower);
		upper.set(lower);

		for (int i = 1; i < m_vertexCount; ++i){
			XForm.mulToOut( xf, m_vertices[i], v);
			//Vec2 v = Mul(xf, m_vertices[i]);
			Vec2.minToOut( lower, v, lower);
			Vec2.maxToOut( upper, v, upper);
		}

		//Vec2 r(m_radius, m_radius);
		//aabb->lowerBound = lower - r;
		//aabb->upperBound = upper + r;

		aabb.lowerBound.x = lower.x - m_radius;
		aabb.lowerBound.y = lower.y - m_radius;
		aabb.upperBound.x = upper.x + m_radius;
		aabb.upperBound.y = upper.y + m_radius;
	}

	// djm pooled
	private static final Vec2 normalL = new Vec2();
	private static final Vec2 intoVec = new Vec2();
	private static final Vec2 outoVec = new Vec2();
	private static final float[] depths = new float[Settings.maxPolygonVertices];
	private static final MassData md = new MassData();
	private static final Vec2 center = new Vec2();

	/**
	 * @see Shape#computeSubmergedArea(Vec2, float, XForm, Vec2)
	 */
	@Override
	public final float computeSubmergedArea(	 final Vec2 normal,
	                                        	 final float offset,
	                                        	 final XForm xf,
	                                        	 final Vec2 c) {
		//Transform plane into shape co-ordinates
		Mat22.mulTransToOut(xf.R, normal, normalL);
		final float offsetL = offset - Vec2.dot(normal,xf.position);

		int diveCount = 0;
		int intoIndex = -1;
		int outoIndex = -1;

		boolean lastSubmerged = false;
		int i;
		for (i = 0; i < m_vertexCount; ++i){
			depths[i] = Vec2.dot(normalL,m_vertices[i]) - offsetL;
			final boolean isSubmerged = depths[i]<-Settings.EPSILON;
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
					XForm.mulToOut( xf, md.center, c);
					return md.mass;
				}
				else{
					//Completely dry
					return 0;
				}
				//break; djm not needed

			case 1:
				if(intoIndex==-1){
					intoIndex = m_vertexCount-1;
				}
				else{
					outoIndex = m_vertexCount-1;
				}
				break;
		}

		final int intoIndex2 = (intoIndex+1) % m_vertexCount;
		final int outoIndex2 = (outoIndex+1) % m_vertexCount;

		final float intoLambda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
		final float outoLambda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);

		intoVec.set(m_vertices[intoIndex].x*(1-intoLambda)+m_vertices[intoIndex2].x*intoLambda,
		            m_vertices[intoIndex].y*(1-intoLambda)+m_vertices[intoIndex2].y*intoLambda);
		outoVec.set(m_vertices[outoIndex].x*(1-outoLambda)+m_vertices[outoIndex2].x*outoLambda,
		            m_vertices[outoIndex].y*(1-outoLambda)+m_vertices[outoIndex2].y*outoLambda);

		// Initialize accumulator
		float area = 0;
		center.setZero();
		p2.set(m_vertices[intoIndex2]);
		Vec2 p3;

		final float k_inv3 = 1.0f / 3.0f;

		// An awkward loop from intoIndex2+1 to outIndex2
		i = intoIndex2;
		while (i != outoIndex2){
			i = (i+1) % m_vertexCount;
			if (i == outoIndex2){
				p3 = outoVec;
			}
			else{
				p3 = m_vertices[i];
			}

			// Add the triangle formed by intoVec,p2,p3
			{
				e1.set(p2).subLocal(intoVec);
				e2.set(p3).subLocal(intoVec);

				final float D = Vec2.cross(e1, e2);

				final float triangleArea = 0.5f * D;

				area += triangleArea;

				// Area weighted centroid
				center.addLocal(intoVec).addLocal(p2).addLocal( p3).mulLocal( triangleArea*k_inv3);
				//center += triangleArea * k_inv3 * (intoVec + p2 + p3);
			}
			//
			p2.set(p3);
		}

		// Normalize and transform centroid
		center.mulLocal(1.0f / area);

		XForm.mulToOut( xf, center, c);

		return area;
	}

	/**
	 * @see Shape#computeSweepRadius(Vec2)
	 */
	@Override
	public final float computeSweepRadius( final Vec2 pivot){
		assert(m_vertexCount > 0);
		float sr = MathUtils.distanceSquared(m_vertices[0], pivot);
		for (int i = 1; i < m_vertexCount; ++i){
			sr = MathUtils.max(sr, MathUtils.distanceSquared(m_vertices[i], pivot));
		}

		return (float) Math.sqrt(sr);
	}

	/**
	 * Get the supporting vertex index in the given direction.
	 * @param d
	 * @return
	 */
	public final int getSupport( final Vec2 d){
		int bestIndex = 0;
		float bestValue = Vec2.dot(m_vertices[0], d);
		for (int i = 1; i < m_vertexCount; ++i){
			final float value = Vec2.dot(m_vertices[i], d);
			if (value > bestValue){
				bestIndex = i;
				bestValue = value;
			}
		}

		return bestIndex;
	}

	/**
	 * Get the supporting vertex in the given direction.
	 * @param d
	 * @return
	 */
	public final Vec2 getSupportVertex( final Vec2 d){
		int bestIndex = 0;
		float bestValue = Vec2.dot(m_vertices[0], d);
		for (int i = 1; i < m_vertexCount; ++i){
			final float value = Vec2.dot(m_vertices[i], d);
			if (value > bestValue){
				bestIndex = i;
				bestValue = value;
			}
		}

		return m_vertices[bestIndex];
	}

	/**
	 * Get the vertex count.
	 * @return
	 */
	public final int getVertexCount()  {
		return m_vertexCount;
	}

	/**
	 * Get a vertex by index.
	 * @param index
	 * @return
	 */
	public final Vec2 getVertex(final int index){
		assert(0 <= index && index < m_vertexCount);
		return m_vertices[index];
	}


	// djm pooling, and from above
	private static final Vec2 p1 = new Vec2();
	private static final Vec2 p2 = new Vec2();
	private static final Vec2 tsd = new Vec2();
	/**
	 * @see Shape#testSegment(XForm, TestSegmentResult, Segment, float)
	 */
	@Override
	public SegmentCollide testSegment(final XForm xf, final TestSegmentResult out, final Segment segment, final float maxLambda){
		float lower = 0.0f, upper = maxLambda;

		p1.set(segment.p1).subLocal( xf.position);
		Mat22.mulTransToOut(xf.R, p1, p1);
		p2.set(segment.p2).subLocal(xf.position);
		Mat22.mulTransToOut(xf.R, p2, p2);
		tsd.set(p2).subLocal(p1);
		int index = -1;

		for (int i = 0; i < m_vertexCount; ++i)
		{
			// p = p1 + a * d
			// dot(normal, p - v) = 0
			// dot(normal, p1 - v) + a * dot(normal, d) = 0
			temp.set( m_vertices[i]).subLocal( p1);
			final float numerator = Vec2.dot(m_normals[i],temp);
			final float denominator = Vec2.dot(m_normals[i], tsd);

			if (denominator == 0.0f)
			{
				if (numerator < 0.0f)
				{
					return SegmentCollide.MISS_COLLIDE;
				}
			}
			else
			{
				// Note: we want this predicate without division:
				// lower < numerator / denominator, where denominator < 0
				// Since denominator < 0, we have to flip the inequality:
				// lower < numerator / denominator <==> denominator * lower > numerator.
				if (denominator < 0.0f && numerator < lower * denominator)
				{
					// Increase lower.
					// The segment enters this half-space.
					lower = numerator / denominator;
					index = i;
				}
				else if (denominator > 0.0f && numerator < upper * denominator)
				{
					// Decrease upper.
					// The segment exits this half-space.
					upper = numerator / denominator;
				}
			}

			if (upper < lower)
			{
				return SegmentCollide.MISS_COLLIDE;
			}
		}

		assert(0.0f <= lower && lower <= maxLambda);

		if (index >= 0)
		{
			out.lambda = lower;
			Mat22.mulToOut(xf.R, m_normals[index], out.normal);
			//*normal = Mul(xf.R, m_normals[index]);
			return SegmentCollide.HIT_COLLIDE;
		}

		out.lambda = 0;
		return SegmentCollide.STARTS_INSIDE_COLLIDE;
	}



	// djm pooled
	private static final Vec2 c = new Vec2();
	private static final Vec2 pRef = new Vec2();
	private static final Vec2 e1 = new Vec2();
	private static final Vec2 e2 = new Vec2();
	/**
	 * @param vs
	 * @return
	 */
	public final void computeCentroidToOut(final Vec2[] vs, final int count, final Vec2 out) {
		assert(count >= 3);

		c.set(0.0f, 0.0f);
		float area = 0.0f;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		pRef.setZero();

		final float inv3 = 1.0f / 3.0f;

		for (int i = 0; i < count; ++i)
		{
			// Triangle vertices.
			final Vec2 p1 = pRef;
			final Vec2 p2 = vs[i];
			final Vec2 p3 = i + 1 < count ? vs[i+1] : vs[0];

			e1.set(p2).subLocal(p1);
			e2.set( p3).subLocal(p1);

			final float D = Vec2.cross(e1, e2);

			final float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid
			c.addLocal(p1).addLocal(p2).addLocal(p3).mulLocal(triangleArea * inv3);
		}

		// Centroid
		assert(area > Settings.EPSILON);
		c.mulLocal(1.0f / area);
		out.set(c);
	}

	/**
	 * @see Shape#computeMass(MassData, float)
	 */
	// djm not very hot method so I'm not pooling.  still optimized though
	@Override
	public void computeMass(final MassData massData, final float density) {
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

		final Vec2 center = new Vec2(0.0f, 0.0f);
		float area = 0.0f;
		float I = 0.0f;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		final Vec2 pRef = new Vec2(0.0f, 0.0f);

		final float k_inv3 = 1.0f / 3.0f;

		final Vec2 e1 = new Vec2();
		final Vec2 e2 = new Vec2();

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
		massData.I = I * density;
	}

	/** Get the local centroid relative to the parent body. */
	public Vec2 getCentroid() {
		return m_centroid.clone();
	}

	/** Get the vertices in local coordinates. */
	public Vec2[] getVertices() {
		return m_vertices;
	}

	/** Get the edge normal vectors.  There is one for each vertex. */
	public Vec2[] getNormals() {
		return m_normals;
	}

	/** Get the centroid and apply the supplied transform. */
	public Vec2 centroid(final XForm xf) {
		return XForm.mul(xf, m_centroid);
	}

	@Override
	public void destructor() {}
}
