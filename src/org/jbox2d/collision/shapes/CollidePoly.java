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
import org.jbox2d.collision.ContactID;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

//Updated to rev 55->108->139 of b2cpp

/** Polygon overlap solver - for internal use. */
public class CollidePoly {
	
	static class ClipVertex {
		public final Vec2 v;

		public final ContactID id;

		public ClipVertex() {
			v = new Vec2();
			id = new ContactID();
		}
	}

	public final int clipSegmentToLine(final ClipVertex vOut[], final ClipVertex vIn[],
	                                          final Vec2 normal, final float offset) {
		// Start with no output points
		int numOut = 0;

		// Calculate the distance of end points to the line
		final float distance0 = Vec2.dot(normal, vIn[0].v) - offset;
		final float distance1 = Vec2.dot(normal, vIn[1].v) - offset;

		// If the points are behind the plane
		if (distance0 <= 0.0f) {
			vOut[numOut] = new ClipVertex();
			vOut[numOut].id.set(vIn[0].id);
			vOut[numOut++].v.set(vIn[0].v);
		}
		if (distance1 <= 0.0f) {
			vOut[numOut] = new ClipVertex();
			vOut[numOut].id.set(vIn[1].id);
			vOut[numOut++].v.set(vIn[1].v);
		}

		// If the points are on different sides of the plane
		if (distance0 * distance1 < 0.0f) {
			// Find intersection point of edge and plane
			final float interp = distance0 / (distance0 - distance1);
			vOut[numOut] = new ClipVertex();
			vOut[numOut].v.x = vIn[0].v.x + interp * (vIn[1].v.x - vIn[0].v.x);
			vOut[numOut].v.y = vIn[0].v.y + interp * (vIn[1].v.y - vIn[0].v.y);

			if (distance0 > 0.0f) {
				vOut[numOut].id.set(vIn[0].id);
			}
			else {
				vOut[numOut].id.set(vIn[1].id);
			}
			++numOut;
		}

		return numOut;
	}

	// djm pooled
	private final Vec2 normal1World = new Vec2();
	public final float edgeSeparation(final PolygonShape poly1, final XForm xf1,
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
	private Vec2 dLocal1 = new Vec2();
	/**
	 * Find the max separation between poly1 and poly2 using face normals
	 * from poly1.
	 * @param poly1
	 * @param xf1
	 * @param poly2
	 * @param xf2
	 * @return
	 */
	public final MaxSeparation findMaxSeparation(final PolygonShape poly1, final XForm xf1,
	                                                    final PolygonShape poly2, final XForm xf2) {
		final MaxSeparation separation = new MaxSeparation();

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
		float s = edgeSeparation(poly1, xf1, edge, poly2, xf2);
		if (s > 0.0f){
			separation.bestSeparation = s;
			return separation;
		}

		// Check the separation for the previous edge normal.
		final int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
		final float sPrev = edgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
		if (sPrev > 0.0f) {
			separation.bestSeparation = sPrev;
			return separation;
		}

		final int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
		final float sNext = edgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
		if (sNext > 0.0f){
			separation.bestSeparation = sNext;
			return separation;
		}

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
			separation.bestFaceIndex = edge;
			separation.bestSeparation = s;
			return separation;
		}

		// Perform a local search for the best edge normal.
		while (true) {
			if (increment == -1) {
				edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
			}
			else {
				edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;
			}

			s = edgeSeparation(poly1, xf1, edge, poly2, xf2);
			if (s > 0.0f) {
				separation.bestSeparation = s;
				return separation;
			}

			if (s > bestSeparation){
				bestEdge = edge;
				bestSeparation = s;
			} else {
				break;
			}
		}

		separation.bestFaceIndex = bestEdge;
		separation.bestSeparation = bestSeparation;

		return separation;
	}

	// djm pooled
	private Vec2 mulTemp = new Vec2();
	private Vec2 normal1 = new Vec2();
	// djm optimized
	public final void findIncidentEdge(final ClipVertex c[],
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

		c[0] = new ClipVertex();
		c[1] = new ClipVertex();

		XForm.mulToOut(xf2, vertices2[i1], c[0].v);
		c[0].id.features.referenceEdge = edge1;
		c[0].id.features.incidentEdge = i1;
		c[0].id.features.incidentVertex = 0;



		XForm.mulToOut(xf2, vertices2[i2], c[1].v);
		c[1].id.features.referenceEdge = edge1;
		c[1].id.features.incidentEdge = i2;
		c[1].id.features.incidentVertex = 1;
	}

	// Find edge normal of max separation on A - return if separating axis is
	// found
	// Find edge normal of max separation on B - return if separation axis is
	// found
	// Choose reference edge as min(minA, minB)
	// Find incident edge
	// Clip

	// The normal points from 1 to 2
	// djm optimized
	public final void collidePolygons(final Manifold manif,
	                                         final PolygonShape polyA, final XForm xfA,
	                                         final PolygonShape polyB, final XForm xfB) {

		//testbed.PTest.debugCount++;
		manif.pointCount = 0; // Fixed a problem with contacts
		final MaxSeparation sepA = findMaxSeparation(polyA, xfA, polyB, xfB);
		if (sepA.bestSeparation > 0.0f) {
			return;
		}

		final MaxSeparation sepB = findMaxSeparation(polyB, xfB, polyA, xfA);
		if (sepB.bestSeparation > 0.0f) {
			return;
		}

		PolygonShape poly1; // reference poly
		PolygonShape poly2; // incident poly
		final XForm xf1 = p_xf1;
		final XForm xf2 = p_xf2;
		int edge1; // reference edge
		byte flip;
		final float k_relativeTol = 0.98f;
		final float k_absoluteTol = 0.001f;

		// TODO_ERIN use "radius" of poly for absolute tolerance.
		if (sepB.bestSeparation > k_relativeTol * sepA.bestSeparation
				+ k_absoluteTol) {
			poly1 = polyB;
			poly2 = polyA;
			xf1.set(xfB);
			xf2.set(xfA);
			edge1 = sepB.bestFaceIndex;
			flip = 1;
		}
		else {
			poly1 = polyA;
			poly2 = polyB;
			xf1.set(xfA);
			xf2.set(xfB);
			edge1 = sepA.bestFaceIndex;
			flip = 0;
		}

		final ClipVertex incidentEdge[] = new ClipVertex[2];
		findIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

		final int count1 = poly1.getVertexCount();
		final Vec2[] vertices1 = poly1.getVertices();

		final Vec2 v11 = vertices1[edge1];
		final Vec2 v12 = edge1 + 1 < count1 ? vertices1[edge1 + 1] : vertices1[0];
		//Vec2 v1 = v12.sub(v11);
		final float v1x = v12.x-v11.x;
		final float v1y = v12.y-v11.y;

		//Vec2 sideNormal = Mat22.mul(xf1.R, v12.sub(v11));
		sideNormal.set(xf1.R.col1.x * v1x + xf1.R.col2.x * v1y,
		               xf1.R.col1.y * v1x + xf1.R.col2.y * v1y);
		sideNormal.normalize();

		//Vec2 frontNormal = Vec2.cross(sideNormal, 1.0f);
		frontNormal.set(sideNormal.y,-sideNormal.x);

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
		final ClipVertex clipPoints1[] = new ClipVertex[2];
		final ClipVertex clipPoints2[] = new ClipVertex[2];
		int np;

		// Clip to box side 1
		np = clipSegmentToLine(clipPoints1, incidentEdge, sideNormal.negate(), sideOffset1);

		if (np < 2) {
			return;
		}

		// Clip to negative box side 1
		np = clipSegmentToLine(clipPoints2, clipPoints1, sideNormal,
		                                   sideOffset2);

		if (np < 2) {
			return;
		}

		// Now clipPoints2 contains the clipped points.
		manif.normal.set(frontNormal);
		if(flip != 0){
			manif.normal.negateLocal();
		}

		int pointCount = 0;
		for (int i = 0; i < Settings.maxManifoldPoints; ++i) {
			final float separation = Vec2.dot(frontNormal, clipPoints2[i].v)
			- frontOffset;

			if (separation <= 0.0f) {
				final ManifoldPoint cp = manif.points[pointCount];
				cp.separation = separation;
				//cp.localPoint1 = XForm.mulT(xfA, clipPoints2[i].v);
				//cp.localPoint2 = XForm.mulT(xfB, clipPoints2[i].v);
				final Vec2 vec = clipPoints2[i].v;
				float u1x = vec.x-xfA.position.x;
				float u1y = vec.y-xfA.position.y;
				cp.localPoint1.x = (u1x * xfA.R.col1.x + u1y * xfA.R.col1.y);
				cp.localPoint1.y = (u1x * xfA.R.col2.x + u1y * xfA.R.col2.y);

				u1x = vec.x-xfB.position.x;
				u1y = vec.y-xfB.position.y;
				cp.localPoint2.x = (u1x * xfB.R.col1.x + u1y * xfB.R.col1.y);
				cp.localPoint2.y = (u1x * xfB.R.col2.x + u1y * xfB.R.col2.y);

				cp.id.set(clipPoints2[i].id);
				cp.id.features.flip = flip;
				++pointCount;
			}
		}

		manif.pointCount = pointCount;

		return;
	}


	// djm pooled
	private final Vec2 colPPc = new Vec2();
	private final Vec2 colPPcLocal = new Vec2();
	private final Vec2 colPPsub = new Vec2();
	private final Vec2 colPPe = new Vec2();
	private final Vec2 colPPp = new Vec2();
	private final Vec2 colPPd = new Vec2();

	/**
	 * puts collision information into the manifold about the collision between a polygon and a point
	 * @param manifold
	 * @param polygon
	 * @param xf1
	 * @param point
	 * @param xf2
	 */
	public final void collidePolygonAndPoint(final Manifold manifold,
	                                                final PolygonShape polygon, final XForm xf1,
	                                                final PointShape point, final XForm xf2) {

		manifold.pointCount = 0;

		// Compute circle position in the frame of the polygon.
		XForm.mulToOut(xf2, point.getMemberLocalPosition(), colPPc);
		XForm.mulTransToOut(xf1, colPPc, colPPcLocal);

		// Find edge with maximum separation.
		int normalIndex = 0;
		float separation = -Float.MAX_VALUE;

		final int vertexCount = polygon.getVertexCount();
		final Vec2[] vertices = polygon.getVertices();
		final Vec2[] normals = polygon.getNormals();
		for (int i = 0; i < vertexCount; ++i) {
			colPPsub.set( colPPcLocal);
			colPPsub.subLocal( vertices[i]);
			final float s = Vec2.dot(normals[i], colPPsub);
			if (s > 0) {
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
			Mat22.mulToOut(xf1.R, normals[normalIndex], manifold.normal);
			manifold.points[0].id.features.incidentEdge = normalIndex;
			manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
			manifold.points[0].id.features.referenceEdge = 0;
			manifold.points[0].id.features.flip = 0;
			final Vec2 position = colPPc;
			XForm.mulTransToOut(xf1, position, manifold.points[0].localPoint1);
			XForm.mulTransToOut(xf2, position, manifold.points[0].localPoint2);
			manifold.points[0].separation = separation;
			return;
		}

		// Project the circle center onto the edge segment.
		final int vertIndex1 = normalIndex;
		final int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		colPPe.set( vertices[vertIndex2]);
		colPPe.subLocal(vertices[vertIndex1]);
		final float length = colPPe.normalize();
		assert(length > Settings.EPSILON);

		// Project the center onto the edge.
		colPPsub.set(colPPcLocal);
		colPPsub.subLocal( vertices[vertIndex1]);
		final float u = Vec2.dot(colPPsub, colPPe);

		colPPp.setZero();
		if (u <= 0.0f) {
			colPPp.set(vertices[vertIndex1]);
			manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
			manifold.points[0].id.features.incidentVertex = vertIndex1;
		}
		else if (u >= length) {
			colPPp.set(vertices[vertIndex2]);
			manifold.points[0].id.features.incidentEdge = Collision.NULL_FEATURE;
			manifold.points[0].id.features.incidentVertex = vertIndex2;
		}
		else {
			colPPp.set(vertices[vertIndex1]);
			colPPp.x += u * colPPe.x;
			colPPp.y += u * colPPe.y;
			manifold.points[0].id.features.incidentEdge = normalIndex;
			manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
		}

		colPPd.set(colPPcLocal);
		colPPd.subLocal( colPPp);

		final float dist = colPPd.normalize();
		if (dist > 0) {
			return;
		}

		manifold.pointCount = 1;

		Mat22.mulToOut(xf1.R, colPPd, manifold.normal);
		final Vec2 position = colPPc;
		XForm.mulTransToOut(xf1, position, manifold.points[0].localPoint1);
		XForm.mulTransToOut(xf2, position, manifold.points[0].localPoint2);
		manifold.points[0].separation = dist;
		manifold.points[0].id.features.referenceEdge = 0;
		manifold.points[0].id.features.flip = 0;
	}

	// djm pooled
	private final Vec2 PEv1 = new Vec2();
	private final Vec2 PEv2 = new Vec2();
	private final Vec2 PEn = new Vec2();
	private final Vec2 PEv1Local = new Vec2();
	private final Vec2 PEv2Local = new Vec2();
	private final Vec2 PEnLocal = new Vec2();
	private final Vec2 temp = new Vec2();
	private final Vec2 temp2 = new Vec2();
	/**
	 * puts collision information into the manifold about a collision between
	 * a polygon and an edge
	 * @param manifold
	 * @param polygon
	 * @param xf1
	 * @param edge
	 * @param xf2
	 */
	public final void collidePolyAndEdge(final Manifold manifold,
	                                            final PolygonShape polygon,
	                                            final XForm xf1,
	                                            final EdgeShape edge,
	                                            final XForm xf2) {
		manifold.pointCount = 0;
		XForm.mulToOut(xf2, edge.getVertex1(), PEv1);
		XForm.mulToOut(xf2, edge.getVertex2(), PEv2);
		Mat22.mulToOut(xf2.R, edge.getNormalVector(), PEn);
		XForm.mulTransToOut(xf1, PEv1, PEv1Local);
		XForm.mulTransToOut(xf1, PEv2, PEv2Local);
		Mat22.mulTransToOut(xf1.R, PEn, PEnLocal);

		float separation1;
		int separationIndex1 = -1; // which normal on the poly found the shallowest depth?
		float separationMax1 = -Float.MAX_VALUE; // the shallowest depth of edge in poly
		float separation2;
		int separationIndex2 = -1; // which normal on the poly found the shallowest depth?
		float separationMax2 = -Float.MAX_VALUE; // the shallowest depth of edge in poly
		float separationMax = -Float.MAX_VALUE; // the shallowest depth of edge in poly
		boolean separationV1 = false; // is the shallowest depth from edge's v1 or v2 vertex?
		int separationIndex = -1; // which normal on the poly found the shallowest depth?

		final int vertexCount = polygon.getVertexCount();
		final Vec2[] vertices = polygon.getVertices();
		final Vec2[] normals = polygon.getNormals();

		int enterStartIndex = -1; // the last poly vertex above the edge
		int enterEndIndex = -1; // the first poly vertex below the edge
		int exitStartIndex = -1; // the last poly vertex below the edge
		int exitEndIndex = -1; // the first poly vertex above the edge
		//int deepestIndex;

		// the "N" in the following variables refers to the edge's normal.
		// these are projections of poly vertices along the edge's normal,
		// a.k.a. they are the separation of the poly from the edge.
		float prevSepN = 0.0f;
		float nextSepN = 0.0f;
		float enterSepN = 0.0f; // the depth of enterEndIndex under the edge (stored as a separation, so it's negative)
		float exitSepN = 0.0f; // the depth of exitStartIndex under the edge (stored as a separation, so it's negative)
		float deepestSepN = Float.MAX_VALUE; // the depth of the deepest poly vertex under the end (stored as a separation, so it's negative)


		// for each poly normal, get the edge's depth into the poly.
		// for each poly vertex, get the vertex's depth into the edge.
		// use these calculations to define the remaining variables declared above.
		temp.set( vertices[vertexCount-1]);
		temp.subLocal( PEv1Local);
		prevSepN = Vec2.dot(temp, PEnLocal);

		for (int i = 0; i < vertexCount; i++) {
			temp.set(PEv1Local);
			temp.subLocal( vertices[i]);
			separation1 = Vec2.dot(temp, normals[i]);
			temp.set(PEv2Local);
			temp.subLocal( vertices[i]);
			separation2 = Vec2.dot(temp, normals[i]);
			if (separation2 < separation1) {
				if (separation2 > separationMax) {
					separationMax = separation2;
					separationV1 = false;
					separationIndex = i;
				}
			} else {
				if (separation1 > separationMax) {
					separationMax = separation1;
					separationV1 = true;
					separationIndex = i;
				}
			}
			if (separation1 > separationMax1) {
				separationMax1 = separation1;
				separationIndex1 = i;
			}
			if (separation2 > separationMax2) {
				separationMax2 = separation2;
				separationIndex2 = i;
			}

			temp.set( vertices[i]);
			temp.subLocal( PEv1Local);

			nextSepN = Vec2.dot(temp, PEnLocal);
			if (nextSepN >= 0.0f && prevSepN < 0.0f) {
				exitStartIndex = (i == 0) ? vertexCount-1 : i-1;
				exitEndIndex = i;
				exitSepN = prevSepN;
			} else if (nextSepN < 0.0f && prevSepN >= 0.0f) {
				enterStartIndex = (i == 0) ? vertexCount-1 : i-1;
				enterEndIndex = i;
				enterSepN = nextSepN;
			}
			if (nextSepN < deepestSepN) {
				deepestSepN = nextSepN;
				//deepestIndex = i;
			}
			prevSepN = nextSepN;
		}

		if (enterStartIndex == -1) {
			// poly is entirely below or entirely above edge, return with no contact:
			return;
		}
		if (separationMax > 0.0f) {
			// poly is laterally disjoint with edge, return with no contact:
			return;
		}

		// if the poly is near a convex corner on the edge
		if ((separationV1 && edge.corner1IsConvex()) || (!separationV1 && edge.corner2IsConvex())) {
			// if shallowest depth was from edge into poly,
			// use the edge's vertex as the contact point:
			if (separationMax > deepestSepN + Settings.linearSlop) {
				// if -normal angle is closer to adjacent edge than this edge,
				// let the adjacent edge handle it and return with no contact:
				if (separationV1) {
					Mat22.mulToOut( xf2.R, edge.getCorner1Vector(), temp);
					Mat22.mulTransToOut(xf1.R, temp, temp);
					if (Vec2.dot(normals[separationIndex1], temp) >= 0.0f) {
						return;
					}
				} else {
					Mat22.mulToOut( xf2.R, edge.getCorner2Vector(), temp);
					Mat22.mulTransToOut(xf1.R, temp, temp);
					if (Vec2.dot(normals[separationIndex2], temp) <= 0.0f) {
						return;
					}
				}

				manifold.pointCount = 1;
				Mat22.mulToOut(xf1.R, normals[separationIndex], manifold.normal);
				manifold.points[0].separation = separationMax;
				manifold.points[0].id.features.incidentEdge = separationIndex;
				manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
				manifold.points[0].id.features.referenceEdge = 0;
				manifold.points[0].id.features.flip = 0;
				if (separationV1) {
					manifold.points[0].localPoint1.set( PEv1Local);
					manifold.points[0].localPoint2.set( edge.getVertex1());
				} else {
					manifold.points[0].localPoint1.set(PEv2Local);
					manifold.points[0].localPoint2.set(edge.getVertex2());
				}
				return;
			}
		}

		// We're going to use the edge's normal now.
		temp.set( PEn);
		temp.mulLocal( -1f);

		manifold.normal.set( temp);// = n.mul(-1.0f);

		// Check whether we only need one contact point.
		if (enterEndIndex == exitStartIndex) {
			manifold.pointCount = 1;
			manifold.points[0].id.features.incidentEdge = enterEndIndex;
			manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
			manifold.points[0].id.features.referenceEdge = 0;
			manifold.points[0].id.features.flip = 0;
			manifold.points[0].localPoint1.set(vertices[enterEndIndex]);
			XForm.mulTransToOut(xf2, XForm.mul(xf1, vertices[enterEndIndex]), manifold.points[0].localPoint2);
			manifold.points[0].separation = enterSepN;
			return;
		}

		manifold.pointCount = 2;

		// dirLocal should be the edge's direction vector, but in the frame of the polygon.
		Vec2.crossToOut(PEnLocal, -1.0f, temp); // TODO: figure out why this optimization didn't work
		//Vec2 dirLocal = XForm.mulT(xf1.R, XForm.mul(xf2.R, edge.GetDirectionVector()));
		temp2.set( vertices[enterEndIndex]);
		temp2.subLocal( PEv1Local);

		final float dirProj1 = Vec2.dot(temp, temp2);
		float dirProj2 = 0.0f;

		// The contact resolution is more robust if the two manifold points are
		// adjacent to each other on the polygon. So pick the first two poly
		// vertices that are under the edge:
		temp2.set( vertices[exitStartIndex]);
		temp2.subLocal( PEv1Local);

		exitEndIndex = (enterEndIndex == vertexCount - 1) ? 0 : enterEndIndex + 1;
		if (exitEndIndex != exitStartIndex) {
			exitStartIndex = exitEndIndex;
			exitSepN = Vec2.dot(PEnLocal, temp2);
		}
		// temp is dirLocal still
		dirProj2 = Vec2.dot(temp, temp2);

		manifold.points[0].id.features.incidentEdge = enterEndIndex;
		manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
		manifold.points[0].id.features.referenceEdge = 0;
		manifold.points[0].id.features.flip = 0;

		if (dirProj1 > edge.getLength()) {
			manifold.points[0].localPoint1.set(PEv2Local);
			manifold.points[0].localPoint2.set(edge.getVertex2());
			final float ratio = (edge.getLength() - dirProj2) / (dirProj1 - dirProj2);
			if (ratio > 100.0f * Settings.EPSILON && ratio < 1.0f) {
				manifold.points[0].separation = exitSepN * (1.0f - ratio) + enterSepN * ratio;
			} else {
				manifold.points[0].separation = enterSepN;
			}
		} else {
			manifold.points[0].localPoint1.set(vertices[enterEndIndex]);
			XForm.mulTransToOut(xf2, XForm.mul(xf1, vertices[enterEndIndex]), manifold.points[0].localPoint2);
			manifold.points[0].separation = enterSepN;
		}

		manifold.points[1].id.features.incidentEdge = exitStartIndex;
		manifold.points[1].id.features.incidentVertex = Collision.NULL_FEATURE;
		manifold.points[1].id.features.referenceEdge = 0;
		manifold.points[1].id.features.flip = 0;

		if (dirProj2 < 0.0f) {
			manifold.points[1].localPoint1.set(PEv1Local);
			manifold.points[1].localPoint2.set(edge.getVertex1());
			final float ratio = (-dirProj1) / (dirProj2 - dirProj1);
			if (ratio > 100.0f * Settings.EPSILON && ratio < 1.0f) {
				manifold.points[1].separation = enterSepN * (1.0f - ratio) + exitSepN * ratio;
			} else {
				manifold.points[1].separation = exitSepN;
			}
		} else {
			manifold.points[1].localPoint1.set(vertices[exitStartIndex]);
			XForm.mulTransToOut(xf2, XForm.mul(xf1, vertices[exitStartIndex]), manifold.points[1].localPoint2);
			manifold.points[1].separation = exitSepN;
		}
	}

	// "Pool" objects
	private final Vec2 sideNormal = new Vec2();
	private final Vec2 frontNormal = new Vec2();

	private final XForm p_xf1 = new XForm();
	private final XForm p_xf2 = new XForm();
}

/** Holder class used internally in CollidePoly */
class MaxSeparation {
	public int bestFaceIndex;
	public float bestSeparation;
}