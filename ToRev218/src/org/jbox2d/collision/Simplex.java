package org.jbox2d.collision;

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.structs.SimplexCache;
import org.jbox2d.collision.structs.SimplexVertex;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

public class Simplex {
	public final SimplexVertex m_v1 = new SimplexVertex();
	public final SimplexVertex m_v2 = new SimplexVertex();
	public final SimplexVertex m_v3 = new SimplexVertex();
	public final SimplexVertex vertices[] = {
			m_v1, m_v2, m_v3
	};
	public int m_count;
	
	public void readCache( SimplexCache cache,
						   Shape shapeA, XForm transformA,
						   Shape shapeB, XForm transformB){
		assert(0 <= cache.count && cache.count <= 3);
		
		// Copy data from cache.
		m_count = cache.count;

		for (int i = 0; i < m_count; ++i){
			SimplexVertex v = vertices[i];
			v.indexA = cache.indexA[i];
			v.indexB = cache.indexB[i];
			Vec2 wALocal = shapeA.getVertex(v.indexA);
			Vec2 wBLocal = shapeB.getVertex(v.indexB);
			XForm.mulToOut(transformA, wALocal, v.wA);
			XForm.mulToOut(transformB, wBLocal, v.wB);
			v.w.set(v.wB).subLocal(v.wA);
			v.a = 0.0f;
		}

		// Compute the new simplex metric, if it is substantially different than
		// old metric then flush the simplex.
		if (m_count > 1){
			float metric1 = cache.metric;
			float metric2 = getMetric();
			if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Settings.EPSILON){
				// Reset the simplex.
				m_count = 0;
			}
		}

		// If the cache is empty or invalid ...
		if (m_count == 0){
			SimplexVertex v = vertices[0];
			v.indexA = 0;
			v.indexB = 0;
			Vec2 wALocal = shapeA.getVertex(0);
			Vec2 wBLocal = shapeB.getVertex(0);
			XForm.mulToOut(transformA, wALocal, v.wA);
			XForm.mulToOut(transformB, wBLocal, v.wB);
			v.w.set(v.wB).subLocal(v.wA);
			m_count = 1;
		}
	}

	public void writeCache(SimplexCache cache){
		cache.metric = getMetric();
		cache.count = (short)m_count;
		for (int i = 0; i < m_count; ++i){
			cache.indexA[i] = (vertices[i].indexA);
			cache.indexB[i] = (vertices[i].indexB);
		}
	}
	
	// djm pooled
	private static final Vec2 case2 = new Vec2();
	private static final Vec2 case22 = new Vec2();
	
	/**
	 * this returns pooled objects. don't keep or modify them
	 * @return
	 */
	public Vec2 getClosestPoint() {
		switch (m_count)
		{
		case 0:
			assert(false);
			return Vec2.zero;

		case 1:
			return m_v1.w;

		case 2:
			case22.set(m_v2.w).mulLocal(m_v2.a);
			case2.set(m_v1.w).mulLocal(m_v1.a).addLocal(case22);
			return case2;

		case 3:
			return Vec2.zero;

		default:
			assert(false);
			return Vec2.zero;
		}
	}

	//djm pooled, and from above
	private static final Vec2 case3 = new Vec2();
	private static final Vec2 case33 = new Vec2();
	public void getWitnessPoints(Vec2 pA, Vec2 pB) {
		switch (m_count)
		{
		case 0:
			assert(false);
			break;

		case 1:
			pA.set(m_v1.wA);
			pB.set(m_v1.wB);
			break;

		case 2:
			case2.set(m_v1.wA).mulLocal(m_v1.a);
			pA.set(m_v2.wA).mulLocal(m_v2.a).addLocal(case2);
			//m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
			//*pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
			case2.set(m_v1.wB).mulLocal(m_v1.a);
			pB.set(m_v2.wB).mulLocal(m_v2.a).addLocal(case2);
			
			break;

		case 3:
			pA.set(m_v1.wA).mulLocal(m_v1.a);
			case3.set(m_v2.wA).mulLocal(m_v2.a);
			case33.set(m_v3.wA).mulLocal(m_v3.a);
			pA.addLocal(case3).addLocal(case33);
			pB.set(pA);
			//*pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
			//*pB = *pA;
			break;

		default:
			assert(false);
			break;
		}
	}
	
	// djm pooled, from above
	public float getMetric(){
		switch (m_count)
		{
		case 0:
			assert(false);
			return 0.0f;
			
		case 1:
			return 0.0f;

		case 2:
			return (float) Math.sqrt(MathUtils.distanceSquared(m_v1.w, m_v2.w));

		case 3:
			case3.set(m_v2.w).subLocal(m_v1.w);
			case33.set(m_v3.w).subLocal(m_v1.w);
			//return Vec2.cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);
			return Vec2.cross(case3,case33);
			
		default:
			assert(false);
			return 0.0f;
		}
	}
	
	// djm pooled
	private static final Vec2 e12 = new Vec2();
	/**
	 * Solve a line segment using barycentric coordinates.
	 */
	public void solve2(){
		Vec2 w1 = m_v1.w;
		Vec2 w2 = m_v2.w;
		e12.set(w2).subLocal(w1);

		// w1 region
		float d12_2 = -Vec2.dot(w1, e12);
		if (d12_2 <= 0.0f){
			// a2 <= 0, so we clamp it to 0
			m_v1.a = 1.0f;
			m_count = 1;
			return;
		}

		// w2 region
		float d12_1 = Vec2.dot(w2, e12);
		if (d12_1 <= 0.0f){
			// a1 <= 0, so we clamp it to 0
			m_v2.a = 1.0f;
			m_count = 1;
			m_v1.set(m_v2);
			return;
		}

		// Must be in e12 region.
		float inv_d12 = 1.0f / (d12_1 + d12_2);
		m_v1.a = d12_1 * inv_d12;
		m_v2.a = d12_2 * inv_d12;
		m_count = 2;
	}
	
	//djm pooled, and from above
	private static final Vec2 e13 = new Vec2();
	private static final Vec2 e23 = new Vec2();

	/**
	 * Solve a line segment using barycentric coordinates.<br/>
	 * Possible regions:<br/>
	 * - points[2]<br/>
 	 * - edge points[0]-points[2]<br/>
	 * - edge points[1]-points[2]<br/>
	 * - inside the triangle
	 */
	public void solve3(){
		Vec2 w1 = m_v1.w;
		Vec2 w2 = m_v2.w;
		Vec2 w3 = m_v3.w;

		// Edge12
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		// a3 = 0
		e12.set(w2).subLocal(w1);
		float w1e12 = Vec2.dot(w1, e12);
		float w2e12 = Vec2.dot(w2, e12);
		float d12_1 = w2e12;
		float d12_2 = -w1e12;

		// Edge13
		// [1      1     ][a1] = [1]
		// [w1.e13 w3.e13][a3] = [0]
		// a2 = 0
		e13.set(w3).subLocal(w1);
		float w1e13 = Vec2.dot(w1, e13);
		float w3e13 = Vec2.dot(w3, e13);
		float d13_1 = w3e13;
		float d13_2 = -w1e13;

		// Edge23
		// [1      1     ][a2] = [1]
		// [w2.e23 w3.e23][a3] = [0]
		// a1 = 0
		e23.set(w3).subLocal(w2);
		float w2e23 = Vec2.dot(w2, e23);
		float w3e23 = Vec2.dot(w3, e23);
		float d23_1 = w3e23;
		float d23_2 = -w2e23;
		
		// Triangle123
		float n123 = Vec2.cross(e12, e13);

		float d123_1 = n123 * Vec2.cross(w2, w3);
		float d123_2 = n123 * Vec2.cross(w3, w1);
		float d123_3 = n123 * Vec2.cross(w1, w2);

		// w1 region
		if (d12_2 <= 0.0f && d13_2 <= 0.0f){
			m_v1.a = 1.0f;
			m_count = 1;
			return;
		}

		// e12
		if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f){
			float inv_d12 = 1.0f / (d12_1 + d12_2);
			m_v1.a = d12_1 * inv_d12;
			m_v2.a = d12_1 * inv_d12;
			m_count = 2;
			return;
		}

		// e13
		if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f){
			float inv_d13 = 1.0f / (d13_1 + d13_2);
			m_v1.a = d13_1 * inv_d13;
			m_v3.a = d13_2 * inv_d13;
			m_count = 2;
			m_v2.set(m_v3);
			return;
		}

		// w2 region
		if (d12_1 <= 0.0f && d23_2 <= 0.0f){
			m_v2.a = 1.0f;
			m_count = 1;
			m_v1.set(m_v2);
			return;
		}

		// w3 region
		if (d13_1 <= 0.0f && d23_1 <= 0.0f)
		{
			m_v3.a = 1.0f;
			m_count = 1;
			m_v1.set(m_v3);
			return;
		}

		// e23
		if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
		{
			float inv_d23 = 1.0f / (d23_1 + d23_2);
			m_v2.a = d23_1 * inv_d23;
			m_v3.a = d23_2 * inv_d23;
			m_count = 2;
			m_v1.set(m_v3);
			return;
		}

		// Must be in triangle123
		float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
		m_v1.a = d123_1 * inv_d123;
		m_v2.a = d123_2 * inv_d123;
		m_v3.a = d123_3 * inv_d123;
		m_count = 3;
	}
}
