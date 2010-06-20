package org.jbox2d.collision;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Sweep;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.SingletonPool;
import org.jbox2d.structs.collision.TOIInput;
import org.jbox2d.structs.collision.distance.DistanceInput;
import org.jbox2d.structs.collision.distance.DistanceOutput;
import org.jbox2d.structs.collision.distance.DistanceProxy;
import org.jbox2d.structs.collision.distance.SimplexCache;

public class TOI {
	public static final int MAX_ITERATIONS = 1000;
	
	public int calls = 0;
	public int iters = 0;
	public int maxIters = 0;
	public int rootIters = 0;
	public int maxRootIters = 0;
	
	//djm pooling
	private final SimplexCache cache = new SimplexCache();
	private final DistanceInput distanceInput = new DistanceInput();
	private final Transform xfA = new Transform();
	private final Transform xfB = new Transform();
	private final DistanceOutput distanceOutput = new DistanceOutput();
	private final SeparationFunction fcn = new SeparationFunction();
	
	/**
	 * Compute the time when two shapes begin to touch or touch at a closer distance.
	 * TOI considers the shape radii. It attempts to have the radii overlap by the tolerance.
	 * Iterations terminate with the overlap is within 0.5 * tolerance. The tolerance should be
	 * smaller than sum of the shape radii.
	 * @warning the sweeps must have the same time interval.
	 * @return the fraction between [0,1] in which the shapes first touch.
	 * fraction=0 means the shapes begin touching/overlapped, and fraction=1 means the shapes don't touch.
	 */
	public final float timeOfImpact(TOIInput input){
		++calls;
		
		final DistanceProxy proxyA = input.proxyA;
		final DistanceProxy proxyB = input.proxyB;
		
		final Sweep sweepA = input.sweepA;
		final Sweep sweepB = input.sweepB;
		
		assert(sweepA.alpha0 == sweepB.alpha0);
		assert(1 - sweepA.alpha0 > Settings.EPSILON);
		
		float radius = proxyA.m_radius + proxyB.m_radius;
		float tolerance = input.tolerance;
		
		float alpha = 0f;
		
		int iter = 0;
		float target = 0f;
		
		cache.count = 0;
		distanceInput.proxyA = input.proxyA;
		distanceInput.proxyB = input.proxyB;
		distanceInput.useRadii = false;
		
		for(;;){
			sweepA.getTransform(xfA, alpha);
			sweepB.getTransform(xfB, alpha);
			
			// Get the distance between shapes.
			distanceInput.transformA.set(xfA);
			distanceInput.transformB.set(xfB);
			SingletonPool.getDistance().distance(distanceOutput, cache, distanceInput);
			
			if(distanceOutput.distance <= 0f){
				alpha = 1f;
				break;
			}
			
			fcn.initialize(cache, proxyA, xfA, proxyB, xfB);
			
			float separation = fcn.evaluate(xfA, xfB);
			if(separation <= 0f){
				alpha = 1f;
				break;
			}
			
			if (iter == 0){
				// Compute a reasonable target distance to give some breathing room
				// for conservative advancement. We take advantage of the shape radii
				// to create additional clearance.
				if (separation > radius){
					target = MathUtils.max(radius - tolerance, 0.75f * radius);
				}
				else{
					target = MathUtils.max(separation - tolerance, 0.02f * radius);
				}
			}
			
			if (separation - target < 0.5f * tolerance){
				if (iter == 0){
					alpha = 1.0f;
					break;
				}

				break;
			}
			
			// Compute 1D root of: f(x) - target = 0
			float newAlpha = alpha;
			{
				float x1 = alpha, x2 = 1.0f;

				float f1 = separation;

				sweepA.getTransform(xfA, x2);
				sweepB.getTransform(xfB, x2);
				float f2 = fcn.evaluate(xfA, xfB);

				// If intervals don't overlap at t2, then we are done.
				if (f2 >= target){
					alpha = 1.0f;
					break;
				}

				// Determine when intervals intersect.
				int rootIterCount = 0;
				for (;;){
					// Use a mix of the secant rule and bisection.
					float x;
					if ( (rootIterCount & 1) == 1){
						// Secant rule to improve convergence.
						x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
					}
					else{
						// Bisection to guarantee progress.
						x = 0.5f * (x1 + x2);
					}

					sweepA.getTransform(xfA, x);
					sweepB.getTransform(xfB, x);

					float f = fcn.evaluate(xfA, xfB);

					if (MathUtils.abs(f - target) < 0.025f * tolerance){
						newAlpha = x;
						break;
					}

					// Ensure we continue to bracket the root.
					if (f > target){
						x1 = x;
						f1 = f;
					}
					else{
						x2 = x;
						f2 = f;
					}

					++rootIterCount;
					++rootIters;

					if (rootIterCount == 50)
					{
						break;
					}
				}

				maxRootIters = MathUtils.max(maxRootIters, rootIterCount);
			}

			// Ensure significant advancement.
			if (newAlpha < (1.0f + 100.0f * Settings.EPSILON) * alpha){
				break;
			}

			alpha = newAlpha;

			++iter;
			++iters;

			if (iter == MAX_ITERATIONS){
				break;
			}
		}

		maxIters = MathUtils.max(maxIters, iter);

		return alpha;		
	}
}

enum Type{
	e_points, e_faceA, e_faceB;
}

class SeparationFunction{
	
	public DistanceProxy m_proxyA;
	public DistanceProxy m_proxyB;
	public Type m_type;
	public final Vec2 m_localPoint = new Vec2();
	public final Vec2 m_axis = new Vec2();
	
	
	// djm pooling
	private final Vec2 localPointA = new Vec2();
	private final Vec2 localPointB = new Vec2();
	private final Vec2 pointA = new Vec2();
	private final Vec2 pointB = new Vec2();
	private final Vec2 localPointA1 = new Vec2();
	private final Vec2 localPointA2 = new Vec2();
	private final Vec2 normal = new Vec2();
	private final Vec2 localPointB1 = new Vec2();
	private final Vec2 localPointB2 = new Vec2();
	private final Vec2 temp = new Vec2();
	private final Vec2 pA = new Vec2();
	private final Vec2 dA = new Vec2();
	private final Vec2 pB = new Vec2();
	private final Vec2 dB = new Vec2();
	private final Vec2 r = new Vec2();

	public void initialize( final SimplexCache cache, final DistanceProxy proxyA, final Transform transformA,
							final DistanceProxy proxyB, final Transform transformB){
		m_proxyA = proxyA;
		m_proxyB = proxyB;
		int count = cache.count;
		assert(0 < count && count < 3);
		
		if(count == 1){
			m_type = Type.e_points;
			/*b2Vec2 localPointA = m_proxyA->GetVertex(cache->indexA[0]);
			b2Vec2 localPointB = m_proxyB->GetVertex(cache->indexB[0]);
			b2Vec2 pointA = b2Mul(transformA, localPointA);
			b2Vec2 pointB = b2Mul(transformB, localPointB);
			m_axis = pointB - pointA;
			m_axis.Normalize();*/
			localPointA.set(m_proxyA.getVertex(cache.indexA[0]));
			localPointB.set(m_proxyA.getVertex(cache.indexB[0]));
			Transform.mulToOut(transformA, localPointA, pointA);
			Transform.mulToOut(transformB, localPointB, pointB);
			m_axis.set(pointB).subLocal(pointA);
			m_axis.normalize();
		}else if(cache.indexB[0] == cache.indexB[1]){
			// Two points on A and one on B
			m_type = Type.e_faceA;
			
			/*b2Vec2 localPointA1 = m_proxyA->GetVertex(cache->indexA[0]);
			b2Vec2 localPointA2 = m_proxyA->GetVertex(cache->indexA[1]);
			b2Vec2 localPointB = m_proxyB->GetVertex(cache->indexB[0]);
			m_localPoint = 0.5f * (localPointA1 + localPointA2);
			m_axis = b2Cross(localPointA2 - localPointA1, 1.0f);
			m_axis.Normalize();*/
			
			localPointA1.set(m_proxyA.getVertex(cache.indexA[0]));
			localPointA2.set(m_proxyA.getVertex(cache.indexA[1]));
			localPointB.set(m_proxyA.getVertex(cache.indexB[0]));
			m_localPoint.set(localPointA1).addLocal(localPointA2).mulLocal(.5f);
			
			m_axis.set(localPointA2).subLocal(localPointA1);
			Vec2.crossToOut(m_axis, 1f, m_axis);
			m_axis.normalize();

			/* normal = b2Mul(transformA.R, m_axis);
			b2Vec2 pointA = b2Mul(transformA, m_localPoint);
			b2Vec2 pointB = b2Mul(transformB, localPointB);*/
			Mat22.mulToOut(transformA.R, m_axis, normal);
			Transform.mulToOut(transformA, m_localPoint, pointA);
			Transform.mulToOut(transformB, localPointB, pointB);
			
			temp.set(pointB).subLocal(pointA);
			float s = Vec2.dot(temp, normal);
			if (s < 0.0f){
				m_axis.negateLocal();
			}
		}else if( cache.indexA[0] == cache.indexA[1]){
			// Two points on B and one on A.
			m_type = Type.e_faceB;
			/*b2Vec2 localPointA = proxyA->GetVertex(cache->indexA[0]);
			b2Vec2 localPointB1 = proxyB->GetVertex(cache->indexB[0]);
			b2Vec2 localPointB2 = proxyB->GetVertex(cache->indexB[1]);
			m_localPoint = 0.5f * (localPointB1 + localPointB2);
			m_axis = b2Cross(localPointB2 - localPointB1, 1.0f);
			m_axis.Normalize();*/
			
			localPointA.set(m_proxyA.getVertex(cache.indexA[0]));
			localPointB1.set(m_proxyA.getVertex(cache.indexB[0]));
			localPointB2.set(m_proxyA.getVertex(cache.indexB[1]));
			m_localPoint.set(localPointA1).addLocal(localPointA2).mulLocal(.5f);
			
			m_axis.set(localPointB2).subLocal(localPointB1);
			Vec2.crossToOut(m_axis, 1f, m_axis);
			m_axis.normalize();

			/*b2Vec2 normal = b2Mul(transformB.R, m_axis);
			b2Vec2 pointB = b2Mul(transformB, m_localPoint);
			b2Vec2 pointA = b2Mul(transformA, localPointA);*/

			Mat22.mulToOut(transformB.R, m_axis, normal);
			Transform.mulToOut(transformB, m_localPoint, pointB);
			Transform.mulToOut(transformA, localPointA, pointA);
			
			temp.set(pointA).subLocal(pointB);
			float s = Vec2.dot(temp, normal);
			if (s < 0.0f){
				m_axis.negateLocal();
			}
		}else{
			// Two points on B and two points on A.
			// The faces are parallel.
			
			/*b2Vec2 localPointA1 = m_proxyA->GetVertex(cache->indexA[0]);
			b2Vec2 localPointA2 = m_proxyA->GetVertex(cache->indexA[1]);
			b2Vec2 localPointB1 = m_proxyB->GetVertex(cache->indexB[0]);
			b2Vec2 localPointB2 = m_proxyB->GetVertex(cache->indexB[1]);*/
			
			localPointA1.set(m_proxyA.getVertex(cache.indexA[0]));
			localPointA2.set(m_proxyA.getVertex(cache.indexA[1]));
			localPointB1.set(m_proxyA.getVertex(cache.indexB[0]));
			localPointB2.set(m_proxyA.getVertex(cache.indexB[1]));
			
			/*b2Vec2 pA = b2Mul(transformA, localPointA1);
			b2Vec2 dA = b2Mul(transformA.R, localPointA2 - localPointA1);
			b2Vec2 pB = b2Mul(transformB, localPointB1);
			b2Vec2 dB = b2Mul(transformB.R, localPointB2 - localPointB1);*/
			
			temp.set(localPointA2).subLocal(localPointA1);
			Transform.mulToOut(transformA, localPointA1, pA);
			Mat22.mulToOut(transformA.R, temp, dA);
			
			temp.set(localPointB2).subLocal(localPointB1);
			Transform.mulToOut(transformB, localPointB1, pB);
			Mat22.mulToOut(transformB.R, temp, dB);

			float a = Vec2.dot(dA, dA);
			float e = Vec2.dot(dB, dB);
			r.set(pA).subLocal(pB);
			float c = Vec2.dot(dA, r);
			float f = Vec2.dot(dB, r);
			
			float b = Vec2.dot(dA, dB);
			float denom = a * e - b * b;

			float s = 0.0f;
			if (denom != 0.0f){
				s = MathUtils.clamp((b * f - c * e) / denom, 0.0f, 1.0f);
			}

			float t = (b * s + f) / e;

			if (t < 0.0f){
				t = 0.0f;
				s = MathUtils.clamp(-c / a, 0.0f, 1.0f);
			}
			else if (t > 1.0f){
				t = 1.0f;
				s = MathUtils.clamp((b - c) / a, 0.0f, 1.0f);
			}
			
			/*b2Vec2 localPointA = localPointA1 + s * (localPointA2 - localPointA1);
			b2Vec2 localPointB = localPointB1 + t * (localPointB2 - localPointB1);*/
			
			localPointA.set(localPointA2).subLocal(localPointA1).mulLocal(s).addLocal(localPointA1);
			localPointB.set(localPointB2).subLocal(localPointB1).mulLocal(t).addLocal(localPointB1);

			if (s == 0.0f || s == 1.0f){
				m_type = Type.e_faceB;
				m_axis.set(localPointB2).subLocal(localPointB1);
				Vec2.crossToOut(m_axis, 1f, m_axis);
				m_axis.normalize();

				m_localPoint.set(localPointB);

				/*b2Vec2 normal = b2Mul(transformB.R, m_axis);
				b2Vec2 pointA = b2Mul(transformA, localPointA);
				b2Vec2 pointB = b2Mul(transformB, localPointB);

				float32 sgn = b2Dot(pointA - pointB, normal);
				if (sgn < 0.0f)
				{
					m_axis = -m_axis;
				}*/
				Mat22.mulToOut(transformB.R, m_axis, normal);
				Transform.mulToOut(transformA, localPointA, pointA);
				Transform.mulToOut(transformB, localPointB, pointB);
				
				temp.set(pointA).subLocal(pointB);
				float sgn = Vec2.dot(temp, normal);
				if (sgn < 0.0f){
					m_axis.negateLocal();
				}
			}
			else
			{
				m_type = Type.e_faceA;
				
				/*m_axis = b2Cross(localPointA2 - localPointA1, 1.0f);
				m_axis.Normalize();

				m_localPoint = localPointA;

				b2Vec2 normal = b2Mul(transformA.R, m_axis);
				b2Vec2 pointA = b2Mul(transformA, localPointA);
				b2Vec2 pointB = b2Mul(transformB, localPointB);

				float32 sgn = b2Dot(pointB - pointA, normal);
				if (sgn < 0.0f)
				{
					m_axis = -m_axis;
				}*/
				
				m_axis.set(localPointA2).subLocal(localPointA1);
				Vec2.crossToOut(m_axis, 1f, m_axis);
				m_axis.normalize();
				
				m_localPoint.set(localPointA);
				
				Mat22.mulToOut(transformA.R, m_axis, normal);
				Transform.mulToOut(transformA, localPointA, pointA);
				Transform.mulToOut(transformB, localPointB, pointB);
				
				temp.set(pointB).subLocal(pointA);
				float sgn = Vec2.dot(temp, normal);
				if (sgn < 0.0f){
					m_axis.negateLocal();
				}
			}
		}
	}
	
	// djm pooled
	private final Vec2 axisA = new Vec2();
	private final Vec2 axisB = new Vec2();;
	public float evaluate(final Transform transformA, final Transform transformB){
		switch(m_type){
		case e_points:{
			/*b2Vec2 axisA = b2MulT(transformA.R,  m_axis);
			b2Vec2 axisB = b2MulT(transformB.R, -m_axis);
			b2Vec2 localPointA = m_proxyA->GetSupportVertex(axisA);
			b2Vec2 localPointB = m_proxyB->GetSupportVertex(axisB);
			b2Vec2 pointA = b2Mul(transformA, localPointA);
			b2Vec2 pointB = b2Mul(transformB, localPointB);
			float32 separation = b2Dot(pointB - pointA, m_axis);
			return separation;*/
			Mat22.mulToOut(transformA.R, m_axis, axisA);
			Mat22.mulToOut(transformB.R, m_axis.negateLocal(), axisB);
			m_axis.negateLocal();
			localPointA.set(m_proxyA.getSupportVertex(axisA));
			localPointB.set(m_proxyB.getSupportVertex(axisB));
			Transform.mulToOut(transformA, localPointA, pointA);
			Transform.mulToOut(transformB, localPointB, pointB);
			
			temp.set(pointB).subLocal(pointA);
			float separation = Vec2.dot(temp, m_axis);
			return separation;
		}
		case e_faceA:{
			
			/*b2Vec2 normal = b2Mul(transformA.R, m_axis);
			b2Vec2 pointA = b2Mul(transformA, m_localPoint);

			b2Vec2 axisB = b2MulT(transformB.R, -normal);

			b2Vec2 localPointB = m_proxyB->GetSupportVertex(axisB);
			b2Vec2 pointB = b2Mul(transformB, localPointB);

			float32 separation = b2Dot(pointB - pointA, normal);
			return separation;*/
			
			Mat22.mulToOut(transformA.R, m_axis, normal);
			Transform.mulToOut(transformA, m_localPoint, pointA);
			
			Mat22.mulTransToOut(transformB.R, normal.negateLocal(), axisB);
			normal.negateLocal();
			
			localPointB.set(m_proxyB.getSupportVertex(axisB));
			Transform.mulToOut(transformB, localPointB, pointB);
			
			temp.set(pointB).subLocal(pointA);
			float separation = Vec2.dot(temp,normal);
			return separation;
		}
		case e_faceB:{
			/*b2Vec2 normal = b2Mul(transformB.R, m_axis);
			b2Vec2 pointB = b2Mul(transformB, m_localPoint);

			b2Vec2 axisA = b2MulT(transformA.R, -normal);

			b2Vec2 localPointA = m_proxyA->GetSupportVertex(axisA);
			b2Vec2 pointA = b2Mul(transformA, localPointA);

			float32 separation = b2Dot(pointA - pointB, normal);
			return separation;*/
			
			Mat22.mulToOut(transformB.R, m_axis, normal);
			Transform.mulToOut(transformB, m_localPoint, pointB);
			
			Mat22.mulTransToOut(transformA.R, normal.negateLocal(), axisA);
			normal.negateLocal();
			
			localPointA.set(m_proxyA.getSupportVertex(axisA));
			Transform.mulToOut(transformA, localPointA, pointA);
			
			temp.set(pointA).subLocal(pointB);
			float separation = Vec2.dot(temp,normal);
			return separation;
		}
		default:
			assert(false);
			return 0f;
		}
	}
}