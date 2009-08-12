package org.jbox2d.structs.collision;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.TLVec2;

public class WorldManifold {
	/**
	 * World vector pointing from A to B
	 */
	public final Vec2 m_normal;
	
	/**
	 * World contact point (point of intersection)
	 */
	public final Vec2[] m_points;
	
	public WorldManifold(){
		m_normal = new Vec2();
		m_points = new Vec2[Settings.maxManifoldPoints];
		for(int i=0; i<Settings.maxManifoldPoints; i++){
			m_points[i] = new Vec2();
		}
	}
	
	
	// djm pooling
	private static final TLVec2 tlpointA = new TLVec2();
	private static final TLVec2 tlpointB = new TLVec2();
	private static final TLVec2 tlnormal = new TLVec2();
	private static final TLVec2 tlcA = new TLVec2();
	private static final TLVec2 tlcB = new TLVec2();
	private static final TLVec2 tlplanePoint = new TLVec2();
	private static final TLVec2 tlclipPoint = new TLVec2();

	
	public final void initialize(final Manifold manifold, final Transform xfA, float radiusA,
	                             final Transform xfB, float radiusB){
		if(manifold.m_pointCount == 0){
			return;
		}
		
		final Vec2 normal = tlnormal.get();
		final Vec2 cA = tlcA.get();
		final Vec2 cB = tlcB.get();
		
		switch(manifold.m_type){
			case e_circles:
				final Vec2 pointA = tlpointA.get();
				final Vec2 pointB = tlpointB.get();
				
				Transform.mulToOut( xfA, manifold.m_localPoint, pointA);
				Transform.mulToOut( xfB, manifold.m_points[0].m_localPoint, pointB);
				normal.set(1,0);
				
				if( MathUtils.distanceSquared( pointA, pointB) > Settings.EPSILON * Settings.EPSILON){
					normal.set(pointB).subLocal( pointA);
					normal.normalize();
				}
				
				m_normal.set(normal);
				
				cA.set(normal).mulLocal( radiusA).addLocal( pointA);
				cB.set(normal).mulLocal( radiusB).addLocal( pointB);
				m_points[0].set( cA).subLocal( cB).mulLocal( 0.5f);
				break;
			case e_faceA:
				{
					final Vec2 planePoint = tlplanePoint.get();
					
					Mat22.mulToOut( xfA.R, manifold.m_localPlaneNormal, normal);
					Transform.mulToOut( xfA, manifold.m_localPoint, planePoint);
					
					m_normal.set( normal);
					
					final Vec2 clipPoint = tlclipPoint.get();
					
					for(int i=0; i<manifold.m_pointCount; i++){
						Transform.mulToOut( xfB, manifold.m_points[i].m_localPoint, clipPoint);
						cA.set(clipPoint).subLocal( planePoint);
						float scalar = radiusA - Vec2.dot( cA, normal);
						cA.set(normal).mulLocal( scalar).addLocal( clipPoint);
						cB.set( normal).mulLocal( radiusB).subLocal( clipPoint).negateLocal();
						m_points[i].set(cA).addLocal( cB).mulLocal( 0.5f);
					}
				}
				break;
			case e_faceB:
				final Vec2 planePoint = tlplanePoint.get();
				
				Mat22.mulToOut( xfB.R, manifold.m_localPlaneNormal, normal);
				Transform.mulToOut( xfB, manifold.m_localPoint, planePoint);
				
				m_normal.set( normal).negateLocal();
				
				final Vec2 clipPoint = tlclipPoint.get();
				
				for(int i=0; i<manifold.m_pointCount; i++){
					Transform.mulToOut( xfA, manifold.m_points[i].m_localPoint, clipPoint);
					cB.set(clipPoint).subLocal( planePoint);
					float scalar = radiusB - Vec2.dot( cB, normal);
					cB.set(normal).mulLocal( scalar).addLocal( clipPoint);
					cA.set( normal).mulLocal( radiusA).subLocal( clipPoint).negateLocal();
					m_points[i].set(cA).addLocal( cB).mulLocal( 0.5f);
				}
				break;
		}
	}
}
