/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
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
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.structs.collision.RayCastInput;
import org.jbox2d.structs.collision.RayCastOutput;

//Updated to rev 100

/** A convex polygon shape. Create using Body.createShape(ShapeDef), not the ructor here. */
public class PolygonShape extends Shape {
	/** Dump lots of debug information. */
	private static boolean m_debug = false;
	
	/**
	 * Local position of the shape centroid in parent body frame.
	 */
	public final Vec2 m_centroid = new Vec2();
	
	/**
	 * The vertices of the shape. Note: use getVertexCount(), not m_vertices.length, to
	 * get number of active vertices.
	 */
	public final Vec2 m_vertices[];
	
	/**
	 * The normals of the shape. Note: use getVertexCount(), not m_normals.length, to get
	 * number of active normals.
	 */
	public final Vec2 m_normals[];
	
	/**
	 * Number of active vertices in the shape.
	 */
	public int m_vertexCount;
	
	// pooling
	private final Vec2 pool1 = new Vec2();
	private final Vec2 pool2 = new Vec2();
	private final Vec2 pool3 = new Vec2();
	private final Vec2 pool4 = new Vec2();
	private final Vec2 pool5 = new Vec2();
	private final Vec2 pool6 = new Vec2();
	private Transform poolt1 = new Transform();
	
	public PolygonShape() {
		m_type = ShapeType.POLYGON;
		
		m_vertexCount = 0;
		m_vertices = new Vec2[Settings.maxPolygonVertices];
		for (int i = 0; i < m_vertices.length; i++) {
			m_vertices[i] = new Vec2();
		}
		m_normals = new Vec2[Settings.maxPolygonVertices];
		for (int i = 0; i < m_normals.length; i++) {
			m_normals[i] = new Vec2();
		}
		m_radius = Settings.polygonRadius;
		m_centroid.setZero();
	}
	
	public final Shape clone() {
		PolygonShape shape = new PolygonShape();
		shape.m_centroid.set(this.m_centroid);
		for (int i = 0; i < shape.m_normals.length; i++) {
			shape.m_normals[i].set(m_normals[i]);
			shape.m_vertices[i].set(m_vertices[i]);
		}
		shape.m_radius = this.m_radius;
		shape.m_vertexCount = this.m_vertexCount;
		return shape;
	}
	
	/**
	 * Get the supporting vertex index in the given direction.
	 * 
	 * @param d
	 * @return
	 */
	public final int getSupport(final Vec2 d) {
		int bestIndex = 0;
		float bestValue = Vec2.dot(m_vertices[0], d);
		for (int i = 1; i < m_vertexCount; i++) {
			float value = Vec2.dot(m_vertices[i], d);
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}
		return bestIndex;
	}
	
	/**
	 * Get the supporting vertex in the given direction.
	 * 
	 * @param d
	 * @return
	 */
	public final Vec2 getSupportVertex(final Vec2 d) {
		int bestIndex = 0;
		float bestValue = Vec2.dot(m_vertices[0], d);
		for (int i = 1; i < m_vertexCount; i++) {
			float value = Vec2.dot(m_vertices[i], d);
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}
		return m_vertices[bestIndex];
	}
	
	/**
	 * Copy vertices. This assumes the vertices define a convex polygon.
	 * It is assumed that the exterior is the the right of each edge.
	 */
	public final void set(final Vec2[] vertices, final int count) {
		assert (2 <= count && count <= Settings.maxPolygonVertices);
		m_vertexCount = count;
		
		// Copy vertices.
		for (int i = 0; i < m_vertexCount; ++i) {
			m_vertices[i] = vertices[i];
		}
		
		final Vec2 edge = pool1;
		
		// Compute normals. Ensure the edges have non-zero length.
		for (int i = 0; i < m_vertexCount; ++i) {
			final int i1 = i;
			final int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
			edge.set(m_vertices[i2]).subLocal(m_vertices[i1]);
			
			assert (edge.lengthSquared() > Settings.EPSILON * Settings.EPSILON);
			Vec2.crossToOut(edge, 1f, m_normals[i]);
			m_normals[i].normalize();
		}
		
		if (m_debug) {
			
			final Vec2 r = pool2;
			
			// Ensure the polygon is convex and the interior
			// is to the left of each edge.
			for (int i = 0; i < m_vertexCount; ++i) {
				final int i1 = i;
				final int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
				edge.set(m_vertices[i2]).subLocal(m_vertices[i1]);
				
				for (int j = 0; j < m_vertexCount; ++j) {
					// Don't check vertices on the current edge.
					if (j == i1 || j == i2) {
						continue;
					}
					
					r.set(m_vertices[j]).subLocal(m_vertices[i1]);
					
					// Your polygon is non-convex (it has an indentation) or
					// has colinear edges.
					final float s = Vec2.cross(edge, r);
					assert (s > 0.0f);
				}
			}
		}
		
		// Compute the polygon centroid.
		computeCentroidToOut(m_vertices, m_vertexCount, m_centroid);
	}
	
	/**
	 * Build vertices to represent an axis-aligned box.
	 * 
	 * @param hx
	 *            the half-width.
	 * @param hy
	 *            the half-height.
	 */
	public final void setAsBox(final float hx, final float hy) {
		m_vertexCount = 4;
		m_vertices[0].set(-hx, -hy);
		m_vertices[1].set(hx, -hy);
		m_vertices[2].set(hx, hy);
		m_vertices[3].set(-hx, hy);
		m_normals[0].set(0.0f, -1.0f);
		m_normals[1].set(1.0f, 0.0f);
		m_normals[2].set(0.0f, 1.0f);
		m_normals[3].set(-1.0f, 0.0f);
		m_centroid.setZero();
	}
	
	/**
	 * Build vertices to represent an oriented box.
	 * 
	 * @param hx
	 *            the half-width.
	 * @param hy
	 *            the half-height.
	 * @param center
	 *            the center of the box in local coordinates.
	 * @param angle
	 *            the rotation of the box in local coordinates.
	 */
	public final void setAsBox(final float hx, final float hy, final Vec2 center, final float angle) {
		m_vertexCount = 4;
		m_vertices[0].set(-hx, -hy);
		m_vertices[1].set(hx, -hy);
		m_vertices[2].set(hx, hy);
		m_vertices[3].set(-hx, hy);
		m_normals[0].set(0.0f, -1.0f);
		m_normals[1].set(1.0f, 0.0f);
		m_normals[2].set(0.0f, 1.0f);
		m_normals[3].set(-1.0f, 0.0f);
		m_centroid.set(center);
		
		final Transform xf = poolt1;
		xf.position.set(center);
		xf.R.set(angle);
		
		// Transform vertices and normals.
		for (int i = 0; i < m_vertexCount; ++i) {
			Transform.mulToOut(xf, m_vertices[i], m_vertices[i]);
			Mat22.mulToOut(xf.R, m_normals[i], m_normals[i]);
		}
	}
	
	/**
	 * Set this as a single edge.
	 * 
	 * @param v1
	 * @param v2
	 */
	public final void setAsEdge(final Vec2 v1, final Vec2 v2) {
		m_vertexCount = 2;
		m_vertices[0].set(v1);
		m_vertices[1].set(v2);
		m_centroid.set(v1).addLocal(v2).mulLocal(0.5f);
		// = 0.5f * (v1 + v2);
		m_normals[0].set(v2).subLocal(v1);
		Vec2.crossToOut(m_normals[0], 1f, m_normals[0]);
		// m_normals[0] = Cross(v2 - v1, 1.0f);
		m_normals[0].normalize();
		m_normals[1].set(m_normals[0]).negateLocal();
	}
	
	/**
	 * @see Shape#testPoint(Transform, Vec2)
	 */
	@Override
	public final boolean testPoint(final Transform xf, final Vec2 p) {
		
		final Vec2 pLocal = pool1;
		
		pLocal.set(p).subLocal(xf.position);
		Mat22.mulTransToOut(xf.R, pLocal, pLocal);
		
		if (m_debug) {
			System.out.println("--testPoint debug--");
			System.out.println("Vertices: ");
			for (int i = 0; i < m_vertexCount; ++i) {
				System.out.println(m_vertices[i]);
			}
			System.out.println("pLocal: " + pLocal);
		}
		
		final Vec2 temp = pool2;
		
		for (int i = 0; i < m_vertexCount; ++i) {
			temp.set(pLocal).subLocal(m_vertices[i]);
			final float dot = Vec2.dot(m_normals[i], temp);
			if (dot > 0.0f) {
				return false;
			}
		}
		
		return true;
	}
	
	/**
	 * @see Shape#computeAABB(AABB, Transform, int)
	 */
	@Override
	public final void computeAABB(final AABB argAabb, final Transform argXf) {
		
		final Vec2 lower = pool1;
		final Vec2 upper = pool2;
		final Vec2 v = pool3;
		
		Transform.mulToOut(argXf, m_vertices[0], lower);
		upper.set(lower);
		
		for (int i = 1; i < m_vertexCount; ++i) {
			Transform.mulToOut(argXf, m_vertices[i], v);
			// Vec2 v = Mul(xf, m_vertices[i]);
			Vec2.minToOut(lower, v, lower);
			Vec2.maxToOut(upper, v, upper);
		}
		
		// Vec2 r(m_radius, m_radius);
		// aabb->lowerBound = lower - r;
		// aabb->upperBound = upper + r;
		
		argAabb.lowerBound.x = lower.x - m_radius;
		argAabb.lowerBound.y = lower.y - m_radius;
		argAabb.upperBound.x = upper.x + m_radius;
		argAabb.upperBound.y = upper.y + m_radius;
	}
	
	// djm pooling, and from above
	/*
	 * private static final TLVec2 tlNormalL = new TLVec2();
	 * private static final TLMassData tlMd = new TLMassData();
	 * private static final FloatArray tldepths = new FloatArray();
	 * private static final TLVec2 tlIntoVec = new TLVec2();
	 * private static final TLVec2 tlOutoVec = new TLVec2();
	 * private static final TLVec2 tlP2b = new TLVec2();
	 * private static final TLVec2 tlP3 = new TLVec2();
	 * private static final TLVec2 tlcenter = new TLVec2();
	 * /*
	 * @see Shape#computeSubmergedArea(Vec2, float, XForm, Vec2)
	 * public float computeSubmergedArea(final Vec2 normal, float offset, Transform xf,
	 * Vec2 c) {
	 * final Vec2 normalL = tlNormalL.get();
	 * final MassData md = tlMd.get();
	 * //Transform plane into shape co-ordinates
	 * Mat22.mulTransToOut(xf.R,normal, normalL);
	 * float offsetL = offset - Vec2.dot(normal,xf.position);
	 * final Float[] depths = tldepths.get(Settings.maxPolygonVertices);
	 * int diveCount = 0;
	 * int intoIndex = -1;
	 * int outoIndex = -1;
	 * boolean lastSubmerged = false;
	 * int i = 0;
	 * for (i = 0; i < m_vertexCount; ++i){
	 * depths[i] = Vec2.dot(normalL,m_vertices[i]) - offsetL;
	 * boolean isSubmerged = depths[i]<-Settings.EPSILON;
	 * if (i > 0){
	 * if (isSubmerged){
	 * if (!lastSubmerged){
	 * intoIndex = i-1;
	 * diveCount++;
	 * }
	 * }
	 * else{
	 * if (lastSubmerged){
	 * outoIndex = i-1;
	 * diveCount++;
	 * }
	 * }
	 * }
	 * lastSubmerged = isSubmerged;
	 * }
	 * switch(diveCount){
	 * case 0:
	 * if (lastSubmerged){
	 * //Completely submerged
	 * computeMass(md, 1.0f);
	 * Transform.mulToOut(xf,md.center, c);
	 * return md.mass;
	 * }
	 * else{
	 * return 0;
	 * }
	 * case 1:
	 * if(intoIndex==-1){
	 * intoIndex = m_vertexCount-1;
	 * }
	 * else{
	 * outoIndex = m_vertexCount-1;
	 * }
	 * break;
	 * }
	 * final Vec2 intoVec = tlIntoVec.get();
	 * final Vec2 outoVec = tlOutoVec.get();
	 * final Vec2 e1 = tle1.get();
	 * final Vec2 e2 = tle2.get();
	 * int intoIndex2 = (intoIndex+1) % m_vertexCount;
	 * int outoIndex2 = (outoIndex+1) % m_vertexCount;
	 * float intoLambda = (0 - depths[intoIndex]) / (depths[intoIndex2] -
	 * depths[intoIndex]);
	 * float outoLambda = (0 - depths[outoIndex]) / (depths[outoIndex2] -
	 * depths[outoIndex]);
	 * intoVec.set(m_vertices[intoIndex].x*(1-intoLambda)+m_vertices[intoIndex2].x*intoLambda
	 * ,
	 * m_vertices[intoIndex].y*(1-intoLambda)+m_vertices[intoIndex2].y*intoLambda);
	 * outoVec.set(m_vertices[outoIndex].x*(1-outoLambda)+m_vertices[outoIndex2].x*outoLambda
	 * ,
	 * m_vertices[outoIndex].y*(1-outoLambda)+m_vertices[outoIndex2].y*outoLambda);
	 * // Initialize accumulator
	 * float area = 0;
	 * final Vec2 center = tlcenter.get();
	 * center.setZero();
	 * final Vec2 p2b = tlP2b.get().set(m_vertices[intoIndex2]);
	 * final Vec2 p3 = tlP3.get();
	 * p3.setZero();
	 * float k_inv3 = 1.0f / 3.0f;
	 * // An awkward loop from intoIndex2+1 to outIndex2
	 * i = intoIndex2;
	 * while (i != outoIndex2){
	 * i = (i+1) % m_vertexCount;
	 * if (i == outoIndex2){
	 * p3.set(outoVec);
	 * }
	 * else{
	 * p3.set(m_vertices[i]);
	 * }
	 * // Add the triangle formed by intoVec,p2,p3
	 * {
	 * e1.set(p2b).subLocal(intoVec);
	 * e2.set(p3).subLocal(intoVec);
	 * float D = Vec2.cross(e1, e2);
	 * float triangleArea = 0.5f * D;
	 * area += triangleArea;
	 * // Area weighted centroid
	 * center.x += triangleArea * k_inv3 * (intoVec.x + p2b.x + p3.x);
	 * center.y += triangleArea * k_inv3 * (intoVec.y + p2b.y + p3.y);
	 * }
	 * //
	 * p2b.set(p3);
	 * }
	 * // Normalize and transform centroid
	 * center.x *= 1.0f / area;
	 * center.y *= 1.0f / area;
	 * Transform.mulToOut(xf, center, c);
	 * return area;
	 * }
	 */

	/*
	 * Get the supporting vertex index in the given direction.
	 * @param d
	 * @return
	 * public final int getSupport( final Vec2 d){
	 * int bestIndex = 0;
	 * float bestValue = Vec2.dot(m_vertices[0], d);
	 * for (int i = 1; i < m_vertexCount; ++i){
	 * final float value = Vec2.dot(m_vertices[i], d);
	 * if (value > bestValue){
	 * bestIndex = i;
	 * bestValue = value;
	 * }
	 * }
	 * return bestIndex;
	 * }
	 * /**
	 * Get the supporting vertex in the given direction.
	 * @param d
	 * @return
	 * public final Vec2 getSupportVertex( final Vec2 d){
	 * int bestIndex = 0;
	 * float bestValue = Vec2.dot(m_vertices[0], d);
	 * for (int i = 1; i < m_vertexCount; ++i){
	 * final float value = Vec2.dot(m_vertices[i], d);
	 * if (value > bestValue){
	 * bestIndex = i;
	 * bestValue = value;
	 * }
	 * }
	 * return m_vertices[bestIndex];
	 * }
	 */

	/**
	 * Get the vertex count.
	 * 
	 * @return
	 */
	public final int getVertexCount() {
		return m_vertexCount;
	}
	
	/**
	 * Get a vertex by index.
	 * 
	 * @param index
	 * @return
	 */
	public final Vec2 getVertex(final int index) {
		assert (0 <= index && index < m_vertexCount);
		return m_vertices[index];
	}

	
	/**
	 * @see org.jbox2d.collision.shapes.Shape#raycast(org.jbox2d.structs.collision.RayCastOutput,
	 *      org.jbox2d.structs.collision.RayCastInput, org.jbox2d.common.Transform, int)
	 */
	@Override
	public final boolean raycast(RayCastOutput argOutput, RayCastInput argInput, Transform argXf) {
		final Vec2 p1 = pool1;
		final Vec2 p2 = pool2;
		final Vec2 d = pool3;
		final Vec2 temp = pool4;
		
		p1.set(argInput.p1).subLocal(argXf.position);
		Mat22.mulTransToOut(argXf.R, p1, p1);
		p2.set(argInput.p2).subLocal(argXf.position);
		Mat22.mulTransToOut(argXf.R, p2, p2);
		d.set(p2).subLocal(p1);
		
		if (m_vertexCount == 2) {
			Vec2 v1 = m_vertices[0];
			Vec2 v2 = m_vertices[1];
			Vec2 normal = m_normals[0];
			
			// q = p1 + t * d
			// dot(normal, q - v1) = 0
			// dot(normal, p1 - v1) + t * dot(normal, d) = 0
			temp.set(v1).subLocal(p1);
			float numerator = Vec2.dot(normal, temp);
			float denominator = Vec2.dot(normal, d);
			
			if (denominator == 0.0f) {
				return false;
			}
			
			float t = numerator / denominator;
			if (t < 0.0f || 1.0f < t) {
				return false;
			}
			
			final Vec2 q = pool5;
			final Vec2 r = pool6;
			
			// Vec2 q = p1 + t * d;
			temp.set(d).mulLocal(t);
			q.set(p1).addLocal(temp);
			
			// q = v1 + s * r
			// s = dot(q - v1, r) / dot(r, r)
			// Vec2 r = v2 - v1;
			r.set(v2).subLocal(v1);
			
			float rr = Vec2.dot(r, r);
			if (rr == 0.0f) {
				return false;
			}
			
			temp.set(q).subLocal(v1);
			float s = Vec2.dot(temp, r) / rr;
			if (s < 0.0f || 1.0f < s) {
				return false;
			}
			
			argOutput.fraction = t;
			if (numerator > 0.0f) {
				// argOutput.normal = -normal;
				argOutput.normal.set(normal).mulLocal(-1);
			}
			else {
				// output.normal = normal;
				argOutput.normal.set(normal);
			}
			return true;
		}
		else {
			
			float lower = 0, upper = argInput.maxFraction;
			
			int index = -1;
			
			for (int i = 0; i < m_vertexCount; ++i) {
				// p = p1 + a * d
				// dot(normal, p - v) = 0
				// dot(normal, p1 - v) + a * dot(normal, d) = 0
				temp.set(m_vertices[i]).subLocal(p1);
				final float numerator = Vec2.dot(m_normals[i], temp);
				final float denominator = Vec2.dot(m_normals[i], d);
				
				if (denominator == 0.0f) {
					if (numerator < 0.0f) {
						return false;
					}
				}
				else {
					// Note: we want this predicate without division:
					// lower < numerator / denominator, where denominator < 0
					// Since denominator < 0, we have to flip the inequality:
					// lower < numerator / denominator <==> denominator * lower >
					// numerator.
					if (denominator < 0.0f && numerator < lower * denominator) {
						// Increase lower.
						// The segment enters this half-space.
						lower = numerator / denominator;
						index = i;
					}
					else if (denominator > 0.0f && numerator < upper * denominator) {
						// Decrease upper.
						// The segment exits this half-space.
						upper = numerator / denominator;
					}
				}
				
				if (upper < lower) {
					return false;
				}
			}
			
			assert (0.0f <= lower && lower <= argInput.maxFraction);
			
			if (index >= 0) {
				argOutput.fraction = lower;
				Mat22.mulToOut(argXf.R, m_normals[index], argOutput.normal);
				// normal = Mul(xf.R, m_normals[index]);
				return true;
			}
		}
		return false;
	}
	
	
	public final void computeCentroidToOut(final Vec2[] vs, final int count, final Vec2 out) {
		assert (count >= 3);
		
		out.set(0.0f, 0.0f);
		float area = 0.0f;
		
		if (count == 2) {
			out.set(vs[0]).addLocal(vs[1]).mulLocal(.5f);
			return;
		}
		
		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		final Vec2 pRef = pool1;
		pRef.setZero();
		
		final Vec2 e1 = pool2;
		final Vec2 e2 = pool3;
		
		final float inv3 = 1.0f / 3.0f;
		
		for (int i = 0; i < count; ++i) {
			// Triangle vertices.
			final Vec2 p1 = pRef;
			final Vec2 p2 = vs[i];
			final Vec2 p3 = i + 1 < count ? vs[i + 1] : vs[0];
			
			e1.set(p2).subLocal(p1);
			e2.set(p3).subLocal(p1);
			
			final float D = Vec2.cross(e1, e2);
			
			final float triangleArea = 0.5f * D;
			area += triangleArea;
			
			// Area weighted centroid
			out.addLocal(p1).addLocal(p2).addLocal(p3).mulLocal(triangleArea * inv3);
		}
		
		// Centroid
		assert (area > Settings.EPSILON);
		out.mulLocal(1.0f / area);
	}
	
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
		
		assert (m_vertexCount >= 2);
		
		// A line segment has zero mass.
		if (m_vertexCount == 2) {
			// massData.center = 0.5f * (m_vertices[0] + m_vertices[1]);
			massData.center.set(m_vertices[0]).addLocal(m_vertices[1]).mulLocal(0.5f);
			massData.mass = 0.0f;
			massData.I = 0.0f;
			return;
		}
		
		final Vec2 center = pool1;
		center.setZero();
		float area = 0.0f;
		float I = 0.0f;
		
		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		final Vec2 pRef = pool2;
		pRef.setZero();
		
		final float k_inv3 = 1.0f / 3.0f;
		
		final Vec2 e1 = pool3;
		final Vec2 e2 = pool4;
		
		for (int i = 0; i < m_vertexCount; ++i) {
			// Triangle vertices.
			final Vec2 p1 = pRef;
			final Vec2 p2 = m_vertices[i];
			final Vec2 p3 = i + 1 < m_vertexCount ? m_vertices[i + 1] : m_vertices[0];
			
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
			
			final float intx2 = k_inv3 * (0.25f * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + 0.5f
					* px * px;
			final float inty2 = k_inv3 * (0.25f * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + 0.5f
					* py * py;
			
			I += D * (intx2 + inty2);
		}
		
		// Total mass
		massData.mass = density * area;
		
		// Center of mass
		assert (area > Settings.EPSILON);
		center.mulLocal(1.0f / area);
		massData.center.set(center);
		
		// Inertia tensor relative to the local origin.
		massData.I = I * density;
	}
	
	/*
	 * Get the local centroid relative to the parent body. /
	 * public Vec2 getCentroid() {
	 * return m_centroid.clone();
	 * }
	 */

	/** Get the vertices in local coordinates. */
	public Vec2[] getVertices() {
		return m_vertices;
	}
	
	/** Get the edge normal vectors. There is one for each vertex. */
	public Vec2[] getNormals() {
		return m_normals;
	}
	
	/** Get the centroid and apply the supplied transform. */
	public Vec2 centroid(final Transform xf) {
		return Transform.mul(xf, m_centroid);
	}
	
	/** Get the centroid and apply the supplied transform. */
	public Vec2 centroidToOut(final Transform xf, final Vec2 out) {
		Transform.mulToOut(xf, m_centroid, out);
		return out;
	}
}
