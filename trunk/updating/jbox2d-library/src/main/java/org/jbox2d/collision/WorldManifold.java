package org.jbox2d.collision;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.TLVec2;

// updated to rev 100
/**
 * This is used to compute the current state of a contact manifold.
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
	
	public WorldManifold(){
		normal = new Vec2();
		points = new Vec2[Settings.maxManifoldPoints];
		for(int i=0; i<Settings.maxManifoldPoints; i++){
			points[i] = new Vec2();
		}
	}
	
	
	// djm pooling
	private static final TLVec2 tlpointA = new TLVec2();
	private static final TLVec2 tlpointB = new TLVec2();
	private static final TLVec2 tlcA = new TLVec2();
	private static final TLVec2 tlcB = new TLVec2();
	private static final TLVec2 tlplanePoint = new TLVec2();
	private static final TLVec2 tlclipPoint = new TLVec2();

	
	public final void initialize(final Manifold manifold, final Transform xfA, float radiusA,
	                             final Transform xfB, float radiusB){
		if(manifold.pointCount == 0){
			return;
		}
		
		final Vec2 cA = tlcA.get();
		final Vec2 cB = tlcB.get();
		
		switch(manifold.type){
			case CIRCLES:
				final Vec2 pointA = tlpointA.get();
				final Vec2 pointB = tlpointB.get();
				
				normal.set(1,0);
				Transform.mulToOut( xfA, manifold.localPoint, pointA);
				Transform.mulToOut( xfB, manifold.points[0].localPoint, pointB);
				
				if( MathUtils.distanceSquared( pointA, pointB) > Settings.EPSILON * Settings.EPSILON){
					normal.set(pointB).subLocal( pointA);
					normal.normalize();
				}
								
				cA.set(normal).mulLocal( radiusA).addLocal( pointA);
				cB.set(normal).mulLocal( radiusB).subLocal( pointB).negateLocal();
				points[0].set( cA).addLocal( cB).mulLocal( 0.5f);
				break;
			case FACE_A:
				{
					final Vec2 planePoint = tlplanePoint.get();
					
					Mat22.mulToOut( xfA.R, manifold.localNormal, normal);
					Transform.mulToOut( xfA, manifold.localPoint, planePoint);
										
					final Vec2 clipPoint = tlclipPoint.get();
					
					for(int i=0; i<manifold.pointCount; i++){
//						b2Vec2 clipPoint = b2Mul(xfB, manifold->points[i].localPoint);
//						b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint, normal)) * normal;
//						b2Vec2 cB = clipPoint - radiusB * normal;
//						points[i] = 0.5f * (cA + cB);
						Transform.mulToOut( xfB, manifold.points[i].localPoint, clipPoint);
						// use cA as temporary for now
						cA.set(clipPoint).subLocal( planePoint);
						float scalar = radiusA - Vec2.dot( cA, normal);
						cA.set( normal).mulLocal( scalar).addLocal( clipPoint);
						cB.set( normal).mulLocal( radiusB).subLocal( clipPoint).negateLocal();
						points[i].set( cA).addLocal( cB).mulLocal( 0.5f);
					}
				}
				break;
			case FACE_B:
				final Vec2 planePoint = tlplanePoint.get();
				
				Mat22.mulToOut( xfB.R, manifold.localNormal, normal);
				Transform.mulToOut( xfB, manifold.localPoint, planePoint);
				
				final Vec2 clipPoint = tlclipPoint.get();
				
				for(int i=0; i<manifold.pointCount; i++){
//					b2Vec2 clipPoint = b2Mul(xfA, manifold->points[i].localPoint);
//					b2Vec2 cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint, normal)) * normal;
//					b2Vec2 cA = clipPoint - radiusA * normal;
//					points[i] = 0.5f * (cA + cB);
					Transform.mulToOut( xfA, manifold.points[i].localPoint, clipPoint);
					cB.set(clipPoint).subLocal( planePoint);
					float scalar = radiusB - Vec2.dot( cB, normal);
					cB.set( normal).mulLocal( scalar).addLocal( clipPoint);
					cA.set( normal).mulLocal( radiusA).subLocal( clipPoint).negateLocal();
					points[i].set(cA).addLocal( cB).mulLocal( 0.5f);
				}
				
				// Ensure normal points from A to B.
				normal.negateLocal();
				break;
		}
	}
}
