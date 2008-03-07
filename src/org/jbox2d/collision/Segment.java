package org.jbox2d.collision;

import org.jbox2d.common.*;

//Updated to rev 108 of b2Collision.h/.cpp
public class Segment {
	
	public Vec2 p1, p2;
	
	/*
	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.4.1
	// x = mu1 * p1 + mu2 * p2
	// mu1 + mu2 = 1 && mu1 >= 0 && mu2 >= 0
	// mu1 = 1 - mu2;
	// x = (1 - mu2) * p1 + mu2 * p2
	//   = p1 + mu2 * (p2 - p1)
	// x = s + a * r (s := start, r := end - start)
	// s + a * r = p1 + mu2 * d (d := p2 - p1)
	// -a * r + mu2 * d = b (b := s - p1)
	// [-r d] * [a; mu2] = b
	// Cramer's rule:
	// denom = det[-r d]
	// a = det[b d] / denom
	// mu2 = det[-r b] / denom
	
	//Currently unimplemented in Java port
	public boolean testSegment(float32* lambda, b2Vec2* normal, const b2Segment& segment, float32 maxLambda) {
		b2Vec2 s = segment.p1;
		b2Vec2 r = segment.p2 - s;
		b2Vec2 d = p2 - p1;
		b2Vec2 n = b2Cross(d, 1.0f);

		const float32 k_slop = 100.0f * FLT_EPSILON;
		float32 denom = -b2Dot(r, n);

		// Cull back facing collision and ignore parallel segments.
		if (denom > k_slop)
		{
			// Does the segment intersect the infinite line associated with this segment?
			b2Vec2 b = s - p1;
			float32 a = b2Dot(b, n);

			if (0.0f <= a && a <= maxLambda * denom)
			{
				float32 mu2 = -r.x * b.y + r.y * b.x;

				// Does the segment intersect this segment?
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop))
				{
					a /= denom;
					n.Normalize();
					*lambda = a;
					*normal = n;
					return true;
				}
			}
		}

		return false;
	}
	*/
}
