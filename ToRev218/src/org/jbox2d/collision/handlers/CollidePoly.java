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

import org.jbox2d.collision.Collision;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.collision.Manifold.ManifoldType;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.structs.ClipVertex;
import org.jbox2d.collision.structs.MaxSeparationResult;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

//Updated to rev 55->108->139 of b2CollidePoly.cpp

/** Polygon overlap solver - for internal use. */
public class CollidePoly {

	// djm pooled
	private final static Vec2 normal1World = new Vec2();
	/**
	 * Find the separation between poly1 and poly2 for a give edge normal on poly1.
	 * @param poly1
	 * @param xf1
	 * @param edge1
	 * @param poly2
	 * @param xf2
	 * @return
	 */
	public final static float edgeSeparation(final PolygonShape poly1, final XForm xf1,
	                                         final int edge1,
	                                         final PolygonShape poly2, final XForm xf2) {

		final int count1 = poly1.getVertexCount();
		final Vec2[] vertices1 = poly1.getVertices();
		final Vec2[] normals1 = poly1.getNormals();

		final int count2 = poly2.getVertexCount();
		final Vec2[] vertices2 = poly2.getVertices();

		assert(0 <= edge1 && edge1 < count1);

		// Convert normal from poly1's frame into poly2's frame.
		Mat22.mulToOut(xf1.R, normals1[edge1], normal1World);
		final float normal1x = Vec2.dot(normal1World, xf2.R.col1);
		final float normal1y = Vec2.dot(normal1World, xf2.R.col2);

		// Find support vertex on poly2 for -normal.
		int index = 0;
		float minDot = Float.MAX_VALUE;
		for (int i = 0; i < count2; ++i) {
			final float dot = vertices2[i].x * normal1x + vertices2[i].y * normal1y;
			//Vec2.dot(poly2.m_vertices[i], normal1);
			if (dot < minDot) {
				minDot = dot;
				index = i;
			}
		}

		//Vec2 v1 = XForm.mul(xf1, poly1.m_vertices[edge1]);
		final Vec2 v = vertices1[edge1];
		final float v1x = xf1.position.x + xf1.R.col1.x * v.x + xf1.R.col2.x * v.y;
		final float v1y = xf1.position.y + xf1.R.col1.y * v.x + xf1.R.col2.y * v.y;
		final Vec2 v3 = vertices2[index];
		//Vec2 v2 = XForm.mul(xf2, poly2.m_vertices[index]);
		final float v2x = xf2.position.x + xf2.R.col1.x * v3.x + xf2.R.col2.x * v3.y;
		final float v2y = xf2.position.y + xf2.R.col1.y * v3.x + xf2.R.col2.y * v3.y;
		//float separation = Vec2.dot(v2.sub(v1), normal1World);
		final float separation = (v2x-v1x) * normal1World.x + (v2y-v1y) * normal1World.y;

		return separation;
	}

	// djm pooled
	private static final Vec2 dLocal1 = new Vec2();
	/**
	 * Find the max separation between poly1 and poly2 using face normals
	 * from poly1.
	 * @param poly1
	 * @param xf1
	 * @param poly2
	 * @param xf2
	 * @return
	 */
	public final static void findMaxSeparation(MaxSeparationResult out,
											   final PolygonShape poly1, final XForm xf1,
	                                           final PolygonShape poly2, final XForm xf2) {

		final int count1 = poly1.getVertexCount();
		final Vec2[] normals1 = poly1.getNormals();

		final Vec2 v = poly1.getCentroid();
		final Vec2 v1 = poly2.getCentroid();

		// Vector pointing from the centroid of poly1 to the centroid of poly2.
		//Vec2 d = XForm.mul(xf2, poly2.m_centroid).subLocal(XForm.mul(xf1, poly1.m_centroid));
		//Vec2 dLocal1 = Mat22.mulT(xf1.R, d);
		final float dx = xf2.position.x + xf2.R.col1.x * v1.x + xf2.R.col2.x * v1.y
		- (xf1.position.x + xf1.R.col1.x * v.x + xf1.R.col2.x * v.y);
		final float dy = xf2.position.y + xf2.R.col1.y * v1.x + xf2.R.col2.y * v1.y
		- (xf1.position.y + xf1.R.col1.y * v.x + xf1.R.col2.y * v.y);
		final Vec2 b = xf1.R.col1;
		final Vec2 b1 = xf1.R.col2;
		dLocal1.x = (dx * b.x + dy * b.y);
		dLocal1.y = (dx * b1.x + dy * b1.y);

		// Find edge normal on poly1 that has the largest projection onto d.
		int edge = 0;
		float maxDot = -Float.MAX_VALUE;
		for (int i = 0; i < count1; ++i) {
			final float dot = Vec2.dot(normals1[i], dLocal1);
			if (dot > maxDot) {
				maxDot = dot;
				edge = i;
			}
		}

		// Get the separation for the edge normal.
		float s = CollidePoly.edgeSeparation(poly1, xf1, edge, poly2, xf2);

		// Check the separation for the previous edge normal.
		final int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
		final float sPrev = CollidePoly.edgeSeparation(poly1, xf1, prevEdge, poly2, xf2);

		final int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
		final float sNext = CollidePoly.edgeSeparation(poly1, xf1, nextEdge, poly2, xf2);

		// Find the best edge and the search direction.
		int bestEdge;
		float bestSeparation;
		int increment;
		if (sPrev > s && sPrev > sNext) {
			increment = -1;
			bestEdge = prevEdge;
			bestSeparation = sPrev;
		}
		else if (sNext > s){
			increment = 1;
			bestEdge = nextEdge;
			bestSeparation = sNext;
		} else {
			out.edgeIndex = edge;
			out.bestSeparation = s;
			return;
		}

		// Perform a local search for the best edge normal.
		while (true) {
			if (increment == -1) {
				edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
			}
			else {
				edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;
			}

			s = CollidePoly.edgeSeparation(poly1, xf1, edge, poly2, xf2);

			if (s > bestSeparation){
				bestEdge = edge;
				bestSeparation = s;
			} else {
				break;
			}
		}

		out.edgeIndex = bestEdge;
		out.bestSeparation = bestSeparation;
	}

	// djm pooled
	private static final Vec2 mulTemp = new Vec2();
	private static final Vec2 normal1 = new Vec2();
	// djm optimized
	public final static void findIncidentEdge(final ClipVertex c[],
	                                          final PolygonShape poly1, final XForm xf1, final int edge1,
	                                          final PolygonShape poly2, final XForm xf2) {

		final int count1 = poly1.getVertexCount();
		final Vec2[] normals1 = poly1.getNormals();

		final int count2 = poly2.getVertexCount();
		final Vec2[] vertices2 = poly2.getVertices();
		final Vec2[] normals2 = poly2.getNormals();

		assert(0 <= edge1 && edge1 < count1);

		// Get the normal of the reference edge in poly2's frame.
		Mat22.mulToOut( xf1.R, normals1[edge1], mulTemp);
		Mat22.mulTransToOut(xf2.R, mulTemp, normal1);

		// Find the incident edge on poly2.
		int index = 0;
		float minDot = Float.MAX_VALUE;
		for (int i = 0; i < count2; ++i) {
			final float dot = Vec2.dot(normal1, normals2[i]);
			if (dot < minDot) {
				minDot = dot;
				index = i;
			}
		}

		// Build the clip vertices for the incident edge.
		final int i1 = index;
		final int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

		XForm.mulToOut(xf2, vertices2[i1], c[0].v);
		c[0].id.features.referenceEdge = edge1;
		c[0].id.features.incidentEdge = i1;
		c[0].id.features.incidentVertex = 0;



		XForm.mulToOut(xf2, vertices2[i2], c[1].v);
		c[1].id.features.referenceEdge = edge1;
		c[1].id.features.incidentEdge = i2;
		c[1].id.features.incidentVertex = 1;
	}

	

	// The normal points from 1 to 2
	// djm pooling
	private static final MaxSeparationResult sepA = new MaxSeparationResult();
	private static final MaxSeparationResult sepB = new MaxSeparationResult();
	private static final ClipVertex incidentEdge[] = new ClipVertex[2];
	private static final Vec2 dv = new Vec2();
	private static final Vec2 localNormal = new Vec2();
	private static final Vec2 planePoint = new Vec2();
	static private final Vec2 temp = new Vec2();
	static private final Vec2 sideNormal = new Vec2();
	static private final Vec2 frontNormal = new Vec2();
	private static final ClipVertex clipPoints1[] = new ClipVertex[2];
	private static final ClipVertex clipPoints2[] = new ClipVertex[2];
	
	// Find edge normal of max separation on A - return if separating axis is
	// found
	// Find edge normal of max separation on B - return if separation axis is
	// found
	// Choose reference edge as min(minA, minB)
	// Find incident edge
	// Clip
	public final static void collidePolygons(final Manifold manif,
	                                         final PolygonShape polyA, final XForm xfA,
	                                         final PolygonShape polyB, final XForm xfB) {

		manif.m_pointCount = 0;
		float totalRadius = polyA.m_radius + polyB.m_radius;

		CollidePoly.findMaxSeparation(sepA, polyA, xfA, polyB, xfB);
		if (sepA.bestSeparation > totalRadius) {
			return;
		}

		CollidePoly.findMaxSeparation(sepB, polyB, xfB, polyA, xfA);
		if (sepB.bestSeparation > totalRadius) {
			return;
		}

		PolygonShape poly1; // reference poly
		PolygonShape poly2; // incident poly
		XForm xf1, xf2;
		int edge1; // reference edge
		byte flip;
		final float k_relativeTol = 0.98f;
		final float k_absoluteTol = 0.001f;

		// TODO_ERIN use "radius" of poly for absolute tolerance.
		if (sepB.bestSeparation > k_relativeTol * sepA.bestSeparation + k_absoluteTol) {
			poly1 = polyB;
			poly2 = polyA;
			xf1 = xfB;
			xf2 = xfA;
			edge1 = sepB.edgeIndex;
			manif.m_type = ManifoldType.e_faceB;
			flip = 1;
		}
		else {
			poly1 = polyA;
			poly2 = polyB;
			xf1 = xfA;
			xf2 = xfB;
			manif.m_type = ManifoldType.e_faceA;
			edge1 = sepA.edgeIndex;
			flip = 0;
		}

		CollidePoly.findIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

		final int count1 = poly1.getVertexCount();
		final Vec2[] vertices1 = poly1.getVertices();

		final Vec2 v11 = vertices1[edge1];
		final Vec2 v12 = edge1 + 1 < count1 ? vertices1[edge1 + 1] : vertices1[0];
		//Vec2 v1 = v12.sub(v11);
		dv.set(v12.x-v11.x, v12.y-v11.y);

		Vec2.crossToOut(dv, 1f, localNormal);
		localNormal.normalize();
		planePoint.set(v11).addLocal(v12).mulLocal(.5f);
		
		//Vec2 sideNormal = Mat22.mul(xf1.R, v12.sub(v11));
		temp.set(v12).subLocal(v11);
		Mat22.mulToOut(xf1.R, temp, sideNormal);
		sideNormal.normalize();

		//Vec2 frontNormal = Vec2.cross(sideNormal, 1.0f);
		Vec2.crossToOut(sideNormal, 1f, frontNormal);

		//v11 = XForm.mul(xf1, v11);
		//v12 = XForm.mul(xf1, v12);
		final float v11x = xf1.position.x + xf1.R.col1.x * v11.x + xf1.R.col2.x * v11.y;
		final float v11y = xf1.position.y + xf1.R.col1.y * v11.x + xf1.R.col2.y * v11.y;
		final float v12x = xf1.position.x + xf1.R.col1.x * v12.x + xf1.R.col2.x * v12.y;
		final float v12y = xf1.position.y + xf1.R.col1.y * v12.x + xf1.R.col2.y * v12.y;

		final float frontOffset = frontNormal.x * v11x + frontNormal.y * v11y;
		final float sideOffset1 = -(sideNormal.x * v11x + sideNormal.y * v11y);
		final float sideOffset2 = sideNormal.x * v12x + sideNormal.y * v12y;

		// Clip incident edge against extruded edge1 side edges.
		int np;

		// Clip to box side 1
		np = Collision.clipSegmentToLine(clipPoints1, incidentEdge, sideNormal.negateLocal(), sideOffset1);
		sideNormal.negateLocal();
		
		if (np < 2) {
			return;
		}

		// Clip to negative box side 1
		np = Collision.clipSegmentToLine(clipPoints2, clipPoints1, sideNormal, sideOffset2);

		if (np < 2) {
			return;
		}

		// Now clipPoints2 contains the clipped points.
		manif.m_localPlaneNormal.set(localNormal);
		manif.m_localPoint.set(planePoint);

		int pointCount = 0;
		for (int i = 0; i < Settings.maxManifoldPoints; ++i) {
			final float separation = Vec2.dot(frontNormal, clipPoints2[i].v) - frontOffset;

			if (separation <= totalRadius) {
				final ManifoldPoint cp = manif.m_points[pointCount];
				XForm.mulTransToOut(xf2, clipPoints2[i].v, cp.m_localPoint);
				cp.m_id.set(clipPoints2[i].id);
				cp.m_id.features.flip = flip;
				++pointCount;
			}
		}

		manif.m_pointCount = pointCount;

		return;
	}
}