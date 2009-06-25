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

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

//  Box2d rev 218 Collision.h/.cpp

/**
 * This is used to compute the current state of a contact manifold.
 */
public class WorldManifold {

	/**
	 * world vector pointing from A to B
	 */
	public final Vec2 m_normal;

	/**
	 * world contact point (point of intersection)
	 */
	public final Vec2 m_points[];

	// djm pooled
	private static final Vec2 pointA = new Vec2();
	private static final Vec2 pointB = new Vec2();
	private static final Vec2 normal = new Vec2();
	private static final Vec2 cA = new Vec2();
	private static final Vec2 cB = new Vec2();
	private static final Vec2 planePoint = new Vec2();
	private static final Vec2 clipPoint = new Vec2();
	private static final Vec2 temp = new Vec2();
	/**
	 * Evaluate the manifold with supplied transforms. This assumes
	 * modest motion from the original state. This does not change the
	 * point count, impulses, etc. The radii must come from the shapes
	 * that generated the manifold.
	 */
	public void Initialize(final Manifold manifold,
	                       final XForm xfA, final float radiusA,
	                       final XForm xfB, final float radiusB){
		if (manifold.m_pointCount == 0){
			return;
		}

		switch (manifold.m_type){
			case e_circles:
				XForm.mulToOut(xfA, manifold.m_localPoint, pointA);
				XForm.mulToOut(xfB, manifold.m_points[0].m_localPoint, pointB);
				normal.set(1.0f, 0.0f);
				if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON){
					normal.set(pointB);
					normal.subLocal(pointA);
					normal.normalize();
				}

				m_normal.set(normal);

				//Vec2 cA = pointA + radiusA * normal;
				//Vec2 cB = pointB - radiusB * normal;
				//m_points[0] = 0.5f * (cA + cB);
				cA.set(normal);
				cA.mulLocal(radiusA);
				cA.addLocal(pointA);

				cB.set( normal);
				cB.mulLocal( radiusB);
				cB.subLocal( pointB).negateLocal();

				m_points[0].set(cA);
				m_points[0].addLocal( cB);
				m_points[0].mulLocal( .5f);
				break;

			case e_faceA:
				Mat22.mulToOut(xfA.R, manifold.m_localPlaneNormal, normal);
				XForm.mulToOut(xfA, manifold.m_localPoint, planePoint);

				// Ensure normal points from A to B.
				m_normal.set(normal);

				for (int i = 0; i < manifold.m_pointCount; ++i)
				{
					XForm.mulToOut(xfB, manifold.m_points[i].m_localPoint, clipPoint);
					//Vec2 cA = clipPoint + (radiusA - Dot(clipPoint - planePoint, normal)) * normal;
					//Vec2 cB = clipPoint - radiusB * normal;
					temp.set(clipPoint).subLocal( planePoint);
					cA.set( normal).mulLocal( radiusA - Vec2.dot( temp, normal));
					cA.addLocal( clipPoint);

					cB.set(normal).mulLocal( radiusB);
					cB.subLocal( clipPoint).negateLocal();

					//m_points[i] = 0.5f * (cA + cB);
					m_points[i].set(cA);
					m_points[i].addLocal( cB);
					m_points[i].mulLocal( .5f);
				}
				break;
			case e_faceB:
				Mat22.mulToOut(xfB.R, manifold.m_localPlaneNormal, normal);
				XForm.mulToOut(xfB, manifold.m_localPoint, planePoint);

				// Ensure normal points from A to B.
				m_normal.set(normal).negateLocal();

				for (int i = 0; i < manifold.m_pointCount; ++i)
				{
					XForm.mulToOut(xfA, manifold.m_points[i].m_localPoint, clipPoint);

					//Vec2 cA = clipPoint - radiusA * normal;
					//Vec2 cB = clipPoint + (radiusB - Dot(clipPoint - planePoint, normal)) * normal;
					temp.set(clipPoint).subLocal( planePoint);
					cA.set( normal).mulLocal( radiusA);
					cA.subLocal( clipPoint).negateLocal();

					cB.set(normal).mulLocal( radiusB - Vec2.dot( temp, normal));
					cB.addLocal( clipPoint);

					//m_points[i] = 0.5f * (cA + cB);
					m_points[i].set(cA);
					m_points[i].addLocal( cB);
					m_points[i].mulLocal( .5f);
				}
				break;
		}
	}

	public WorldManifold(){
		m_points = new Vec2[Settings.maxManifoldPoints];
		for(int i=0; i<m_points.length; i++){
			m_points[i] = new Vec2();
		}
		m_normal = new Vec2();
	}

	public void set(final WorldManifold wm){
		for(int i=0; i<m_points.length; i++){
			m_points[i].set(wm.m_points[i]);
		}
		m_normal.set(wm.m_normal);
	}
}
