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

import java.util.List;

import org.jbox2d.common.*;


//Updated to rev 56->132 of b2Shape.cpp/.h / b2PolygonShape.cpp/.h

public class PolygonShape extends Shape implements SupportsGenericDistance{
    // Dump lots of debug information
	private static boolean m_debug = true;
    
	/** Local position of the shape centroid in parent body frame. */
    public Vec2 m_centroid;
    
    /** The oriented bounding box of the shape. */
    public OBB m_obb;

    /** The vertices of the shape.  Note: use getVertexCount(), not m_vertices.length, to get number of active vertices. */
    public Vec2 m_vertices[];
    /** The normals of the shape.  Note: use getVertexCount(), not m_normals.length, to get number of active normals. */
    public Vec2 m_normals[];
    /** The normals of the shape.  Note: use getVertexCount(), not m_coreVertices.length, to get number of active vertices. */
    public Vec2 m_coreVertices[];
    /** Number of active vertices in the shape. */
    public int m_vertexCount;
    
    public PolygonShape(ShapeDef def) {
    	super(def);
    	
    	assert(def.type == ShapeType.POLYGON_SHAPE);
    	m_type = ShapeType.POLYGON_SHAPE;
    	PolygonDef poly = (PolygonDef)def;

    	m_vertexCount = poly.vertexCount();
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
    	for (int i = 0; i < m_vertexCount; ++i) {
    		int i1 = i;
    		int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
    		Vec2 edge = m_vertices[i2].sub(m_vertices[i1]);
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
	    			float s = Vec2.dot(m_normals[i], m_vertices[j].sub(m_vertices[i]));
	    			assert(s < -Settings.linearSlop);
	    		}
	    	}
	
	    	// Ensure the polygon is counter-clockwise.
	    	for (int i = 1; i < m_vertexCount; ++i) {
	    		float cross = Vec2.cross(m_normals[i-1], m_normals[i]);
	
	    		// Keep asinf happy.
	    		cross = MathUtils.clamp(cross, -1.0f, 1.0f);
	
	    		// You have consecutive edges that are almost parallel on your polygon.
	    		float angle = (float)Math.asin(cross);
	    		assert(angle > Settings.angularSlop);
	    	}
	    	//#endif
    	}

    	// Compute the polygon centroid.
    	m_centroid = computeCentroid(poly.vertices);

    	// Compute the oriented bounding box.
    	computeOBB(m_obb, m_vertices);

    	// Create core polygon shape by shifting edges inward.
    	// Also compute the min/max radius for CCD.
    	for (int i = 0; i < m_vertexCount; ++i) {
    		int i1 = i - 1 >= 0 ? i - 1 : m_vertexCount - 1;
    		int i2 = i;

    		Vec2 n1 = m_normals[i1];
    		Vec2 n2 = m_normals[i2];
    		Vec2 v = m_vertices[i].sub(m_centroid);

    		Vec2 d = new Vec2();
    		d.x = Vec2.dot(n1, v) - Settings.toiSlop;
    		d.y = Vec2.dot(n2, v) - Settings.toiSlop;

    		// Shifting the edge inward by b2_toiSlop should
    		// not cause the plane to pass the centroid.

    		// Your shape has a radius/extent less than b2_toiSlop.
    		if (m_debug && (d.x < 0.0f || d.y < 0.0f)) {
    			System.out.println("Error, dumping details: ");
    			System.out.println("d.x: "+d.x+"d.y: "+d.y);
    			System.out.println("n1: "+n1+"; n2: "+n2);
    			System.out.println("v: "+v);
    		}
    		assert(d.x >= 0.0f);
    		assert(d.y >= 0.0f);
    		Mat22 A = new Mat22();
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
    
    public void updateSweepRadius(Vec2 center) {
    	// Update the sweep radius (maximum radius) as measured from
    	// a local center point.
    	m_sweepRadius = 0.0f;
    	for (int i = 0; i < m_vertexCount; ++i) {
    		Vec2 d = m_coreVertices[i].sub(center);
    		m_sweepRadius = Math.max(m_sweepRadius, d.length());
    	}
    }
    
    public boolean testPoint(XForm xf, Vec2 p) {
    	Vec2 pLocal = Mat22.mulT(xf.R, p.sub(xf.position));

    	if (m_debug) {
    		System.out.println("--testPoint debug--");
    		System.out.println("Vertices: ");
    		for (int i=0; i < m_vertexCount; ++i) {
    			System.out.println(m_vertices[i]);
    		}
    		System.out.println("pLocal: "+pLocal);
    	}
    	
    	for (int i = 0; i < m_vertexCount; ++i) {
    		float dot = Vec2.dot(m_normals[i], pLocal.sub(m_vertices[i]));
    		
    		if (dot > 0.0f) {
    			return false;
    		}
    	}

    	return true;
    }
    
    
//    bool b2PolygonShape::TestSegment(
//    		const b2XForm& xf,
//    		float32* lambda,
//    		b2Vec2* normal,
//    		const b2Segment& segment,
//    		float32 maxLambda) const
//    	{
//    		float32 lower = 0.0f, upper = maxLambda;
//
//    		b2Vec2 p1 = b2MulT(xf.R, segment.p1 - xf.position);
//    		b2Vec2 p2 = b2MulT(xf.R, segment.p2 - xf.position);
//    		b2Vec2 d = p2 - p1;
//    		int32 index = -1;
//
//    		for (int32 i = 0; i < m_vertexCount; ++i)
//    		{
//    			// p = p1 + a * d
//    			// dot(normal, p - v) = 0
//    			// dot(normal, p1 - v) + a * dot(normal, d) = 0
//    			float32 numerator = b2Dot(m_normals[i], m_vertices[i] - p1);
//    			float32 denominator = b2Dot(m_normals[i], d);
//
//    			if (denominator < 0.0f && numerator > lower * denominator)
//    			{
//    				// The segment enters this half-space.
//    				lower = numerator / denominator;
//    				index = i;
//    			}
//    			else if (denominator > 0.0f && numerator < upper * denominator)
//    			{
//    				// The segment exits this half-space.
//    				upper = numerator / denominator;
//    			}
//
//    			if (upper < lower)
//    			{
//    				return false;
//    			}
//    		}
//
//    		b2Assert(0.0f <= lower && lower <= maxLambda);
//
//    		if (index >= 0)
//    		{
//    			*lambda = lower;
//    			*normal = b2Mul(xf.R, m_normals[index]);
//    			return true;
//    		}
//
//    		return false;
//    	}
    
    
    public Vec2 centroid(XForm xf) {
    	return XForm.mul(xf, m_centroid);
    }

    public Vec2 support(XForm xf, Vec2 d) {
        Vec2 dLocal = Mat22.mulT(xf.R, d);

        int bestIndex = 0;
        float bestValue = Vec2.dot(m_coreVertices[0], dLocal);
        for (int i = 1; i < m_vertexCount; ++i) {
            float value = Vec2.dot(m_coreVertices[i], dLocal);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }

        return XForm.mul(xf, m_coreVertices[bestIndex]); 
    }

	public static Vec2 computeCentroid(List<Vec2> vs) {
			int count = vs.size();
	        assert(count >= 3);
	
	        Vec2 c = new Vec2();
	        float area = 0.0f;
	
	        // pRef is the reference point for forming triangles.
	        // It's location doesn't change the result (except for rounding error).
	        Vec2 pRef = new Vec2();
	//    #if 0
	//        // This code would put the reference point inside the polygon.
	//        for (int32 i = 0; i < count; ++i)
	//        {
	//            pRef += vs[i];
	//        }
	//        pRef *= 1.0f / count;
	//    #endif
	
	        final float inv3 = 1.0f / 3.0f;
	
	        for (int i = 0; i < count; ++i) {
	            // Triangle vertices.
	            Vec2 p1 = pRef;
	            Vec2 p2 = vs.get(i);
	            Vec2 p3 = i + 1 < count ? vs.get(i+1) : vs.get(0);
	
	            Vec2 e1 = p2.sub(p1);
	            Vec2 e2 = p3.sub(p1);
	
	            float D = Vec2.cross(e1, e2);
	
	            float triangleArea = 0.5f * D;
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
	
	// http://www.geometrictools.com/Documentation/MinimumAreaRectangle.pdf
	public static void computeOBB(OBB obb, Vec2[] vs){
		int count = vs.length;
		assert(count <= Settings.maxPolygonVertices);
		Vec2[] p = new Vec2[Settings.maxPolygonVertices + 1];
		for (int i = 0; i < count; ++i){
			p[i] = vs[i];
		}
		p[count] = p[0];

		float minArea = Float.MAX_VALUE;
		
		for (int i = 1; i <= count; ++i){
			Vec2 root = p[i-1];
			Vec2 ux = p[i].sub(root);
			float length = ux.normalize();
			assert(length > Settings.EPSILON);
			Vec2 uy = new Vec2(-ux.y, ux.x);
			Vec2 lower = new Vec2(Float.MAX_VALUE, Float.MAX_VALUE);
			Vec2 upper = new Vec2(-Float.MAX_VALUE, -Float.MAX_VALUE);

			for (int j = 0; j < count; ++j) {
				Vec2 d = p[j].sub(root);
				Vec2 r = new Vec2();
				r.x = Vec2.dot(ux, d);
				r.y = Vec2.dot(uy, d);
				lower = Vec2.min(lower, r);
				upper = Vec2.max(upper, r);
			}

			float area = (upper.x - lower.x) * (upper.y - lower.y);
			if (area < 0.95f * minArea){
				minArea = area;
				obb.R.col1.set(ux);
				obb.R.col2.set(uy);
				Vec2 center = new Vec2(0.5f * (lower.x + upper.x), 0.5f * (lower.y + upper.y));
				obb.center = root.add(Mat22.mul(obb.R, center));
				obb.extents = new Vec2(0.5f * (upper.x - lower.x), 0.5f * (upper.y - lower.y));
			}
		}

		assert(minArea < Float.MAX_VALUE);
	}
	
	public void computeAABB(AABB aabb, XForm xf) {
		Mat22 R = Mat22.mul(xf.R, m_obb.R);
		Mat22 absR = Mat22.abs(R);
		Vec2 h = Mat22.mul(absR, m_obb.extents);
		Vec2 position = xf.position.add(Mat22.mul(xf.R, m_obb.center));
		aabb.lowerBound = position.sub(h);
		aabb.upperBound = position.add(h);//save a Vec2 creation, reuse temp
	}
	
	public void computeSweptAABB(AABB aabb, XForm transform1, XForm transform2) {
		
		AABB aabb1 = new AABB();
		AABB aabb2 = new AABB();
		computeAABB(aabb1, transform1);
		computeAABB(aabb2, transform2);
		aabb.lowerBound = Vec2.min(aabb1.lowerBound, aabb2.lowerBound);
		aabb.upperBound = Vec2.max(aabb1.upperBound, aabb2.upperBound);
		//System.out.println("poly sweepaabb: "+aabb.lowerBound+" "+aabb.upperBound);
	}
	
	public void computeMass(MassData massData) {
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

		Vec2 center = new Vec2(0.0f, 0.0f);
		float area = 0.0f;
		float I = 0.0f;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		Vec2 pRef = new Vec2(0.0f, 0.0f);

		float k_inv3 = 1.0f / 3.0f;

		for (int i = 0; i < m_vertexCount; ++i) {
			// Triangle vertices.
			Vec2 p1 = pRef;
			Vec2 p2 = m_vertices[i];
			Vec2 p3 = i + 1 < m_vertexCount ? m_vertices[i+1] : m_vertices[0];

			Vec2 e1 = p2.sub(p1);
			Vec2 e2 = p3.sub(p1);

			float D = Vec2.cross(e1, e2);

			float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid
			center.x += triangleArea * k_inv3 * (p1.x + p2.x + p3.x);
			center.y += triangleArea * k_inv3 * (p1.y + p2.y + p3.y);

			float px = p1.x, py = p1.y;
			float ex1 = e1.x, ey1 = e1.y;
			float ex2 = e2.x, ey2 = e2.y;

			float intx2 = k_inv3 * (0.25f * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5f*px*px;
			float inty2 = k_inv3 * (0.25f * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5f*py*py;

			I += D * (intx2 + inty2);
		}

		// Total mass
		massData.mass = m_density * area;

		// Center of mass
		assert(area > Settings.EPSILON);
		center.mulLocal(1.0f / area);
		massData.center = center.clone();
		
		// Inertia tensor relative to the local origin.
		massData.I = I*m_density;
	}
	
	public Vec2 getFirstVertex(XForm xf) {
		return XForm.mul(xf, m_coreVertices[0]);
	}

	public OBB getOBB() {
		return m_obb.clone();
	}

	public Vec2 getCentroid() {
		return m_centroid.clone();
	}

	public int getVertexCount() {
		return m_vertexCount;
	}

	public Vec2[] getVertices() {
		return m_vertices;
	}

	public Vec2[] getCoreVertices()	{
		return m_coreVertices;
	}

}
