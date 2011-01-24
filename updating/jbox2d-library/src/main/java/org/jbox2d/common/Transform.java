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
 * A transform contains translation and rotation. It is used to represent
 * the position and orientation of rigid frames.
 */
public class Transform {
	
	/** The translation caused by the transform */
	public final Vec2 position;
	
	/** A matrix representing a rotation */
	public final Mat22 R;
	
	/** The default constructor. */
	public Transform() {
		position = new Vec2();
		R = new Mat22();
	}
	
	/** Initialize as a copy of another transform. */
	public Transform(final Transform xf) {
		position = xf.position.clone();
		R = xf.R.clone();
	}
	
	/** Initialize using a position vector and a rotation matrix. */
	public Transform(final Vec2 _position, final Mat22 _R) {
		position = _position.clone();
		R = _R.clone();
	}
	
	/** Set this to equal another transform. */
	public final Transform set(final Transform xf) {
		position.set(xf.position);
		R.set(xf.R);
		return this;
	}
	
	/**
	 * Set this based on the position and angle.
	 * 
	 * @param p
	 * @param angle
	 */
	public final void set(Vec2 p, float angle) {
		position.set(p);
		R.set(angle);
	}
	
	/**
	 * Calculate the angle that the rotation matrix represents.
	 */
	public final float getAngle() {
		return MathUtils.atan2(R.col1.y, R.col1.x);
	}
	
	/** Set this to the identity transform. */
	public final void setIdentity() {
		position.setZero();
		R.setIdentity();
	}
	
	public final static Vec2 mul(final Transform T, final Vec2 v) {
		return new Vec2(T.position.x + T.R.col1.x * v.x + T.R.col2.x * v.y, T.position.y + T.R.col1.y * v.x
				+ T.R.col2.y * v.y);
	}
	
	/* djm added */
	public final static void mulToOut(final Transform T, final Vec2 v, final Vec2 out) {
		final float tempy = T.position.y + T.R.col1.y * v.x + T.R.col2.y * v.y;
		out.x = T.position.x + T.R.col1.x * v.x + T.R.col2.x * v.y;
		out.y = tempy;
	}
	
	public final static Vec2 mulTrans(final Transform T, final Vec2 v) {
		final float v1x = v.x - T.position.x;
		final float v1y = v.y - T.position.y;
		final Vec2 b = T.R.col1;
		final Vec2 b1 = T.R.col2;
		return new Vec2((v1x * b.x + v1y * b.y), (v1x * b1.x + v1y * b1.y));
		// return T.R.mulT(v.sub(T.position));
	}
	
	public final static void mulTransToOut(final Transform T, final Vec2 v, final Vec2 out) {
		final float v1x = v.x - T.position.x;
		final float v1y = v.y - T.position.y;
		final Vec2 b = T.R.col1;
		final Vec2 b1 = T.R.col2;
		final float tempy = v1x * b1.x + v1y * b1.y;
		out.x = v1x * b.x + v1y * b.y;
		out.y = tempy;
	}
	
	@Override
	public final String toString() {
		String s = "XForm:\n";
		s += "Position: " + position + "\n";
		s += "R: \n" + R + "\n";
		return s;
	}
}
