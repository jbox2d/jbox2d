package org.jbox2d.common;

public class Sweep {
	// / This describes the motion of a body/shape for TOI computation.
	// / Shapes are defined with respect to the body origin, which may
	// / no coincide with the center of mass. However, to support dynamics
	// / we must interpolate the center of mass position.

	public Vec2 localCenter; // /< local center of mass position
	public Vec2 c0, c; // /< center world positions
	public float a0, a; // /< world angles
	public float t0; // /< time interval = [t0,1], where t0 is in [0,1]

	public Sweep() {
		localCenter = new Vec2();
		c0 = new Vec2();
		c = new Vec2();
	}

	// / Get the interpolated transform at a specific time.
	// / @param t the normalized time in [0,1].
	public void getXForm(XForm xf, float t) {
		if (xf == null)
			xf = new XForm();
		// center = p + R * localCenter
		if (1.0f - t0 > Settings.EPSILON) {
			float alpha = (t - t0) / (1.0f - t0);
			xf.position.x = (1.0f - alpha) * c0.x + alpha * c.x;
			xf.position.y = (1.0f - alpha) * c0.y + alpha * c.y;
			float angle = (1.0f - alpha) * a0 + alpha * a;
			xf.R.set(angle);
		} else {
			xf.position.set(c);
			xf.R.set(a);
		}

		// Shift to origin
		xf.position.subLocal(Mat22.mul(xf.R, localCenter));
	}

	// / Advance the sweep forward, yielding a new initial state.
	// / @param t the new initial time.
	public void advance(float t) {
		if (t0 < t && 1.0f - t0 > Settings.EPSILON) {
			float alpha = (t - t0) / (1.0f - t0);
			c0.x = (1.0f - alpha) * c0.x + alpha * c.x;
			c0.y = (1.0f - alpha) * c0.y + alpha * c.y;
			a0 = (1.0f - alpha) * a0 + alpha * a;
			t0 = t;
		}
	}

}
