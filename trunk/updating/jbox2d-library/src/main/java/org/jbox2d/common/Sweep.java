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

package org.jbox2d.common;

// updated to rev 100

/**
 * This describes the motion of a body/shape for TOI computation.
 * Shapes are defined with respect to the body origin, which may
 * no coincide with the center of mass. However, to support dynamics
 * we must interpolate the center of mass position.
 */
public class Sweep {
	
	/** Local center of mass position */
	public final Vec2 localCenter;
	/** Center world positions */
	public final Vec2 c0, c;
	/** World angles */
	public float a0, a;
	
	public String toString() {
		String s = "Sweep:\nlocalCenter: " + localCenter + "\n";
		s += "c0: " + c0 + ", c: " + c + "\n";
		s += "a0: " + a0 + ", a: " + a + "\n";
		return s;
	}
	
	public Sweep() {
		localCenter = new Vec2();
		c0 = new Vec2();
		c = new Vec2();
	}
	
	public final void normalize() {
		float d = MathUtils.TWOPI * MathUtils.floor(a0 / MathUtils.TWOPI);
		a0 -= d;
		a -= d;
	}
	
	public final Sweep set(Sweep argCloneFrom) {
		localCenter.set(argCloneFrom.localCenter);
		c0.set(argCloneFrom.c0);
		c.set(argCloneFrom.c);
		a0 = argCloneFrom.a0;
		a = argCloneFrom.a;
		return this;
	}
	
	/**
	 * Get the interpolated transform at a specific time.
	 * 
	 * @param xf
	 *            the result is placed here - must not be null
	 * @param t
	 *            the normalized time in [0,1].
	 */
	public final void getTransform(final Transform xf, final float alpha) {
		assert (xf != null);
		// if (xf == null)
		// xf = new XForm();
		// center = p + R * localCenter
		/*
		 * if (1.0f - t0 > Settings.EPSILON) {
		 * float alpha = (t - t0) / (1.0f - t0);
		 * xf.position.x = (1.0f - alpha) * c0.x + alpha * c.x;
		 * xf.position.y = (1.0f - alpha) * c0.y + alpha * c.y;
		 * float angle = (1.0f - alpha) * a0 + alpha * a;
		 * xf.R.set(angle);
		 * } else {
		 * xf.position.set(c);
		 * xf.R.set(a);
		 * }
		 */

		xf.position.x = (1.0f - alpha) * c0.x + alpha * c.x;
		xf.position.y = (1.0f - alpha) * c0.y + alpha * c.y;
		// float angle = (1.0f - alpha) * a0 + alpha * a;
		// xf.R.set(angle);
		xf.R.set((1.0f - alpha) * a0 + alpha * a);
		
		// Shift to origin
		// xf.position.subLocal(Mat22.mul(xf.R, localCenter));
		xf.position.x -= xf.R.col1.x * localCenter.x + xf.R.col2.x * localCenter.y;
		xf.position.y -= xf.R.col1.y * localCenter.x + xf.R.col2.y * localCenter.y;
	}
	
	/**
	 * Advance the sweep forward, yielding a new initial state.
	 * 
	 * @param t
	 *            the new initial time.
	 */
	public final void advance(final float t) {
		// c0 = (1.0f - t) * c0 + t*c;
		c0.x = (1.0f - t) * c0.x + t * c.x;
		c0.y = (1.0f - t) * c0.y + t * c.y;
		a0 = (1.0f - t) * a0 + t * a;
	}
	
}
