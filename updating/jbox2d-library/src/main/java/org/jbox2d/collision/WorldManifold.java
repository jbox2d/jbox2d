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
package org.jbox2d.collision;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;

// updated to rev 100
/**
 * This is used to compute the current state of a contact manifold.
 * 
 * @author daniel
 */
public class WorldManifold {
	/**
	 * World vector pointing from A to B
	 */
	public final Vec2 normal;
	
	/**
	 * World contact point (point of intersection)
	 */
	public final Vec2[] points;
	
	public WorldManifold() {
		normal = new Vec2();
		points = new Vec2[Settings.maxManifoldPoints];
		for (int i = 0; i < Settings.maxManifoldPoints; i++) {
			points[i] = new Vec2();
		}
	}
	
	private final Vec2 pool1 = new Vec2();
	private final Vec2 pool2 = new Vec2();
	private final Vec2 pool3 = new Vec2();
	private final Vec2 pool4 = new Vec2();
	
	public final void initialize(final Manifold manifold, final Transform xfA, float radiusA, final Transform xfB,
			float radiusB) {
		if (manifold.pointCount == 0) {
			return;
		}
		
		final Vec2 cA = pool1;
		final Vec2 cB = pool2;
		
		switch (manifold.type) {
			case CIRCLES :
				final Vec2 pointA = pool3;
				final Vec2 pointB = pool4;
				
				normal.set(1, 0);
				Transform.mulToOut(xfA, manifold.localPoint, pointA);
				Transform.mulToOut(xfB, manifold.points[0].localPoint, pointB);
				
				if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON) {
					normal.set(pointB).subLocal(pointA);
					normal.normalize();
				}
				
				cA.set(normal).mulLocal(radiusA).addLocal(pointA);
				cB.set(normal).mulLocal(radiusB).subLocal(pointB).negateLocal();
				points[0].set(cA).addLocal(cB).mulLocal(0.5f);
				break;
			case FACE_A : {
				final Vec2 planePoint = pool3;
				
				Mat22.mulToOut(xfA.R, manifold.localNormal, normal);
				Transform.mulToOut(xfA, manifold.localPoint, planePoint);
				
				final Vec2 clipPoint = pool4;
				
				for (int i = 0; i < manifold.pointCount; i++) {
					// b2Vec2 clipPoint = b2Mul(xfB, manifold->points[i].localPoint);
					// b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint,
					// normal)) * normal;
					// b2Vec2 cB = clipPoint - radiusB * normal;
					// points[i] = 0.5f * (cA + cB);
					Transform.mulToOut(xfB, manifold.points[i].localPoint, clipPoint);
					// use cA as temporary for now
					cA.set(clipPoint).subLocal(planePoint);
					float scalar = radiusA - Vec2.dot(cA, normal);
					cA.set(normal).mulLocal(scalar).addLocal(clipPoint);
					cB.set(normal).mulLocal(radiusB).subLocal(clipPoint).negateLocal();
					points[i].set(cA).addLocal(cB).mulLocal(0.5f);
				}
			}
				break;
			case FACE_B :
				final Vec2 planePoint = pool3;
				
				Mat22.mulToOut(xfB.R, manifold.localNormal, normal);
				Transform.mulToOut(xfB, manifold.localPoint, planePoint);
				
				final Vec2 clipPoint = pool4;
				
				for (int i = 0; i < manifold.pointCount; i++) {
					// b2Vec2 clipPoint = b2Mul(xfA, manifold->points[i].localPoint);
					// b2Vec2 cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint,
					// normal)) * normal;
					// b2Vec2 cA = clipPoint - radiusA * normal;
					// points[i] = 0.5f * (cA + cB);
					Transform.mulToOut(xfA, manifold.points[i].localPoint, clipPoint);
					cB.set(clipPoint).subLocal(planePoint);
					float scalar = radiusB - Vec2.dot(cB, normal);
					cB.set(normal).mulLocal(scalar).addLocal(clipPoint);
					cA.set(normal).mulLocal(radiusA).subLocal(clipPoint).negateLocal();
					points[i].set(cA).addLocal(cB).mulLocal(0.5f);
				}
				
				// Ensure normal points from A to B.
				normal.negateLocal();
				break;
		}
	}
}
