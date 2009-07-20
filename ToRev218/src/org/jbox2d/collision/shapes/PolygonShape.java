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
import org.jbox2d.collision.OBB;
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

/** A convex polygon shape.  Create using Body.createShape(ShapeDef), not the constructor here. */
public final class PolygonShape extends Shape{
	/** Dump lots of debug information. */
	private static boolean m_debug = false;


	/**
	 * Local position of the shape centroid in parent body frame.
	 */
	protected final Vec2 m_centroid = new Vec2();

	protected final OBB m_obb = new OBB();

	/**
	 * The vertices of the shape.  Note: use m_vertexCount, not m_vertices.length, to get number of active vertices.
	 */
	protected final Vec2 m_vertices[];

	/**
	 * The vertices of the shape from the local perspective.  Note: use m_vertexCount, not m_vertices.length, to get number of active vertices.
	 */
	protected final Vec2 m_coreVertices[];

	/**
	 * The normals of the shape.  Note: use m_vertexCount, not m_normals.length, to get number of active normals.
	 */
	protected final Vec2 m_normals[];

	/**
	 * Number of active vertices in the shape.
	 */
	protected int m_vertexCount;

	public PolygonShape(final ShapeDef def) {
		super(def);
		assert(def.type == ShapeType.POLYGON_SHAPE);
		final PolygonDef polyDef = (PolygonDef) def;

		m_type = ShapeType.POLYGON_SHAPE;

		m_vertices = new Vec2[Settings.maxPolygonVertices];
		m_normals = new Vec2[Settings.maxPolygonVertices];
		m_coreVertices = new Vec2[Settings.maxPolygonVertices];

		for(int i=0; i<Settings.maxPolygonVertices; i++){
			m_vertices[i] = new Vec2();
			m_normals[i] = new Vec2();
			m_coreVertices[i] = new Vec2();
		}

		set(polyDef.vertices, polyDef.vertexCount);
	}


	// djm pooling
	private static final Vec2 setEdge = new Vec2();
	private static final Vec2 setV = new Vec2();
	private static final Vec2 setD = new Vec2();
	private static final Mat22 setA = new Mat22();
	/**
	 * Copy vertices. This assumes the vertices define a convex polygon.
	 * It is assumed that the exterior is the the right of each edge.
	 */
	public final void set( final Vec2[] vertices, final int count){
		assert(3 <= count && count <= Settings.maxPolygonVertices);
		m_vertexCount = count;

		// Copy vertices.
		for(int i=0; i<m_vertexCount; i++){
			m_vertices[i].set( vertices[i]);
		}

		// Compute normals. Ensure the edges have non-zero length.
		for (int i = 0; i < m_vertexCount; ++i){
			final int i1 = i;
			final int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
			setEdge.set(m_vertices[i2]).subLocal(m_vertices[i1]);
			assert(setEdge.lengthSquared() > Settings.EPSILON * Settings.EPSILON);
			Vec2.crossToOut( setEdge, 1.0f, m_normals[i]);
			m_normals[i].normalize();
		}

		if(m_debug){
			// Ensure the polygon is convex.
			for (int i = 0; i < m_vertexCount; ++i){
				for (int j = 0; j < m_vertexCount; ++j){
					// Don't check vertices on the current edge.
					if (j == i || j == (i + 1) % m_vertexCount){
						continue;
					}

					// Your polygon is non-convex (it has an indentation).
					// Or your polygon is too skinny.
					final float s = Vec2.dot(m_normals[i], m_vertices[j].sub(m_vertices[i]));
					assert(s < -Settings.linearSlop);
				}
			}

			// Ensure the polygon is counter-clockwise.
			for (int i = 1; i < m_vertexCount; ++i){
				float cross = Vec2.cross(m_normals[i-1], m_normals[i]);

				// Keep asinf happy.
				cross = MathUtils.clamp(cross, -1.0f, 1.0f);

				// You have consecutive edges that are almost parallel on your polygon.
				final float angle = (float) Math.asin(cross);
				assert(angle > Settings.angularSlop);
			}
		}

		// Compute the polygon centroid.
		computeCentroidToOut(vertices, count, m_centroid);

		// Compute the oriented bounding box.
		computeOBB(m_obb, m_vertices, m_vertexCount);

		// Create core polygon shape by shifting edges inward.
		// Also compute the min/max radius for CCD.
		for (int i = 0; i < m_vertexCount; ++i){
			final int i1 = i - 1 >= 0 ? i - 1 : m_vertexCount - 1;
			final int i2 = i;

			final Vec2 n1 = m_normals[i1];
			final Vec2 n2 = m_normals[i2];
			setV.set(m_vertices[i]).subLocal(m_centroid);

			setD.x = Vec2.dot(n1, setV) - Settings.toiSlop;
			setD.y = Vec2.dot(n2, setV) - Settings.toiSlop;

			// Shifting the edge inward by b2_toiSlop should
			// not cause the plane to pass the centroid.

			// Your shape has a radius/extent less than b2_toiSlop.
			assert(setD.x >= 0.0f);
			assert(setD.y >= 0.0f);
			setA.col1.x = n1.x; setA.col2.x = n1.y;
			setA.col1.y = n2.x; setA.col2.y = n2.y;
			m_coreVertices[i].set(setA.solve(setD)).addLocal(m_centroid);
		}
	}

	// djm pooling
	private static final Vec2 pLocal = new Vec2();
	private static final Vec2 temp = new Vec2();
	/**
	 * @see Shape#testPoint(XForm, Vec2)
	 */
	@Override
	public final boolean testPoint( final XForm xf,  final Vec2 p){
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

	// djm pooling, and from above
	private static final Vec2 p1 = new Vec2();
	private static final Vec2 p2 = new Vec2();
	private static final Vec2 tsd = new Vec2();
	/**
	 * @see Shape#testSegment(XForm, TestSegmentResult, Segment, float)
	 */
	@Override
	public final SegmentCollide testSegment(final XForm xf, final TestSegmentResult out, final Segment segment, final float maxLambda){
		float lower = 0.0f, upper = maxLambda;

		p1.set(segment.p1).subLocal( xf.position);
		Mat22.mulTransToOut(xf.R, p1, p1);
		p2.set(segment.p2).subLocal(xf.position);
		Mat22.mulTransToOut(xf.R, p2, p2);
		tsd.set(p2).subLocal(p1);
		int index = -1;

		for (int i = 0; i < m_vertexCount; ++i){
			// p = p1 + a * d
			// dot(normal, p - v) = 0
			// dot(normal, p1 - v) + a * dot(normal, d) = 0
			temp.set( m_vertices[i]).subLocal( p1);
			final float numerator = Vec2.dot(m_normals[i],temp);
			final float denominator = Vec2.dot(m_normals[i], tsd);

			if (denominator == 0.0f){
				if (numerator < 0.0f){
					return SegmentCollide.MISS_COLLIDE;
				}
			}
			else{
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
			}

			if (upper < lower){
				return SegmentCollide.MISS_COLLIDE;
			}
		}

		assert(0.0f <= lower && lower <= maxLambda);

		if (index >= 0){
			out.lambda = lower;
			Mat22.mulToOut(xf.R, m_normals[index], out.normal);
			//*normal = Mul(xf.R, m_normals[index]);
			return SegmentCollide.HIT_COLLIDE;
		}

		out.lambda = 0;
		return SegmentCollide.STARTS_INSIDE_COLLIDE;
	}


	// djm pooling
	private static final Mat22 R = new Mat22();
	private static final Mat22 absR = new Mat22();
	private static final Vec2 h = new Vec2();
	private static final Vec2 position = new Vec2();

	/**
	 * @see Shape#computeAABB(AABB, XForm)
	 */
	@Override
	public final void computeAABB(final AABB aabb,  final XForm xf){
		Mat22.mulToOut( xf.R, m_obb.R, R);
		Mat22.absToOut( R, absR);

		Mat22.mulToOut( absR, m_obb.extents, h);
		Mat22.mulToOut(xf.R, m_obb.center, position);
		position.addLocal(xf.position);


		aabb.lowerBound.x = position.x - h.x;
		aabb.lowerBound.y = position.y - h.y;
		aabb.upperBound.x = position.x + h.x;
		aabb.upperBound.y = position.y + h.y;
	}

	// djm pooled
	private static final AABB aabb1 = new AABB();
	private static final AABB aabb2 = new AABB();


	@Override
	public final void computeSweptAABB( final AABB aabb, final XForm transform1, final XForm transform2){
		computeAABB( aabb1, transform1);
		computeAABB( aabb2, transform2);
		Vec2.minToOut( aabb1.lowerBound, aabb2.lowerBound, aabb.lowerBound);
		Vec2.maxToOut( aabb1.upperBound, aabb2.upperBound, aabb.upperBound);
	}

	// djm pooling, from below
	/**
	 * @see Shape#computeMass(MassData, float)
	 */
	@Override
	public final void computeMass(final MassData massData, final float density) {
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

		center.setZero();
		float area = 0.0f;
		float I = 0.0f;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		pRef.setZero();

		final float k_inv3 = 1.0f / 3.0f;

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
	 * Get the oriented bounding box relative to the parent body.
	 * @return
	 */
	public final OBB getOBB(){
		return m_obb;
	}


	/**
	 * Get the local centroid relative to the parent body.
	 * Don't edit.
	 */
	public final Vec2 getCentroid() {
		return m_centroid;
	}

	/**
	 * Get the vertex count.
	 * @return
	 */
	public final int getVertexCount()  {
		return m_vertexCount;
	}

	/** Get the vertices in local coordinates. */
	public final Vec2[] getVertices() {
		return m_vertices;
	}


	/**
	 * Get the core vertices in local coordinates. These vertices
	 * represent a smaller polygon that is used for time of impact
	 * computations.
	 * @return
	 */
	public final Vec2[] getCoreVertices(){
		return m_coreVertices;
	}

	/** Get the edge normal vectors.  There is one for each vertex. */
	public final Vec2[] getNormals() {
		return m_normals;
	}

	/**
	 * Get the first vertex and apply the supplied transform
	 * @param xf
	 * @return
	 */
	public final Vec2 getFirstVertex(final XForm xf){
		return XForm.mul( xf, m_coreVertices[0]);
	}

	/**
	 * Get the first vertex and apply the supplied transform
	 * @param xf
	 * @param out where the result is put
	 */
	public final void getFirstVertexToOut(final XForm xf, final Vec2 out){
		XForm.mulToOut( xf, m_coreVertices[0], out);
	}

	/** Get the centroid and apply the supplied transform. */
	public final Vec2 centroid(final XForm xf) {
		return XForm.mul(xf, m_centroid);
	}

	/** Get the centroid and apply the supplied transform. */
	public final void centroidToOut(final XForm xf, final Vec2 out) {
		XForm.mulToOut(xf, m_centroid, out);
	}

	// djm pooling
	private static final Vec2 dLocal = new Vec2();
	/**
	 * Get the support point in the given world direction.
	 * Use the supplied transform.
	 * @param xf
	 * @param d
	 * @return
	 */
	public final Vec2 support(final XForm xf, final Vec2 d){
		Mat22.mulTransToOut( xf.R, d, dLocal);

		int bestIndex = 0;
		float bestValue = Vec2.dot(m_coreVertices[0], dLocal);
		for (int i = 1; i < m_vertexCount; ++i){
			final float value = Vec2.dot(m_coreVertices[i], dLocal);
			if (value > bestValue){
				bestIndex = i;
				bestValue = value;
			}
		}

		return XForm.mul(xf, m_coreVertices[bestIndex]);
	}

	/**
	 * Get the support point in the given world direction.
	 * Use the supplied transform.
	 * @param xf
	 * @param d
	 * @param out where to put the result
	 * @return
	 */
	public final void supportToOut(final XForm xf, final Vec2 d, final Vec2 out){
		Mat22.mulTransToOut( xf.R, d, dLocal);

		int bestIndex = 0;
		float bestValue = Vec2.dot(m_coreVertices[0], dLocal);
		for (int i = 1; i < m_vertexCount; ++i){
			final float value = Vec2.dot(m_coreVertices[i], dLocal);
			if (value > bestValue){
				bestIndex = i;
				bestValue = value;
			}
		}

		XForm.mulToOut(xf, m_coreVertices[bestIndex], out);
	}

	@Override
	protected final void updateSweepRadius(final Vec2 center){
		// Update the sweep radius (maximum radius) as measured from
		// a local center point.
		m_sweepRadius = 0.0f;
		float dx,dy;
		for (int i = 0; i < m_vertexCount; ++i){
			dx = m_coreVertices[i].x - center.x;
			dy = m_coreVertices[i].y - center.y;
			m_sweepRadius = MathUtils.max(m_sweepRadius, (float)Math.sqrt(dx*dx + dy*dy));
		}
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
	public static final void computeCentroidToOut(final Vec2[] vs, final int count, final Vec2 out) {
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

	 
	private static final Vec2 ux = new Vec2();
	private static final Vec2 uy = new Vec2();
	private static final Vec2 lower = new Vec2();
	private static final Vec2 upper = new Vec2();
	private static final Vec2 d = new Vec2();
	private static final Vec2 r = new Vec2();
	/**
	 * http://www.geometrictools.com/Documentation/MinimumAreaRectangle.pdf
	 * @param obb
	 * @param vs
	 * @param count
	 */
	public static final void computeOBB(final OBB obb, final Vec2[] vs, final int count){
		assert(count <= Settings.maxPolygonVertices);
		final Vec2[] p = new Vec2[Settings.maxPolygonVertices + 1];
		for (int i = 0; i < count; ++i){
			p[i] = vs[i];
		}
		p[count] = p[0];

		float minArea = Float.MAX_VALUE;

		for (int i = 1; i <= count; ++i){
			final Vec2 root = p[i-1];
			ux.set(p[i]);
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
				d.set(p[j]);
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

				final Vec2 center = new Vec2(0.5f * (lower.x + upper.x), 0.5f * (lower.y + upper.y));
				Mat22.mulToOut(obb.R, center, obb.center);
				obb.center.addLocal(root);
				//obb.center = root.add(Mat22.mul(obb.R, center));

				obb.extents.x = 0.5f * (upper.x - lower.x);
				obb.extents.y = 0.5f * (upper.y - lower.y);
			}
		}

		assert(minArea < Float.MAX_VALUE);
	}
}
