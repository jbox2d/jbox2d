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

// created from build 218
public class Vec3 {
	public float x, y, z;

	public Vec3() {
		x = y = z = 0f;
	}

	public Vec3( final float argX, final float argY, final float argZ) {
		x = argX;
		y = argY;
		z = argZ;
	}

	public Vec3( final Vec3 argCopy) {
		x = argCopy.x;
		y = argCopy.y;
		z = argCopy.z;
	}

	public void set( final Vec3 argVec) {
		x = argVec.x;
		y = argVec.y;
		z = argVec.z;
	}

	public Vec3 addLocal( final Vec3 argVec) {
		x += argVec.x;
		y += argVec.y;
		z += argVec.z;
		return this;
	}

	public Vec3 add( final Vec3 argVec) {
		return new Vec3( x + argVec.x, y + argVec.y, z + argVec.z);
	}

	public Vec3 subLocal( final Vec3 argVec) {
		x -= argVec.x;
		y -= argVec.y;
		z -= argVec.z;
		return this;
	}

	public Vec3 sub( final Vec3 argVec) {
		return new Vec3( x - argVec.x, y - argVec.y, z - argVec.z);
	}

	public Vec3 mulLocal( final float argScalar) {
		x *= argScalar;
		y *= argScalar;
		z *= argScalar;
		return this;
	}

	public Vec3 mul( final float argScalar) {
		return new Vec3( x * argScalar, y * argScalar, z * argScalar);
	}

	public Vec3 negate() {
		return new Vec3( -x, -y, -z);
	}

	public Vec3 negateLocal() {
		x *= -1;
		y *= -1;
		z *= -1;
		return this;
	}

	public void setZero() {
		x = 0;
		y = 0;
		z = 0;
	}

	@Override
	public Vec3 clone() {
		return new Vec3( this);
	}

	@Override
	public boolean equals( final Object argOther) {
		if ( argOther instanceof Vec3) {
			final Vec3 other = (Vec3) argOther;
			return other.x == x && other.y == y && other.z == z;
		}
		return false;
	}

	public final static float dot( final Vec3 a, final Vec3 b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	public final static Vec3 cross( final Vec3 a, final Vec3 b) {
		return new Vec3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}

	public final static void crossToOut( final Vec3 a, final Vec3 b, final Vec3 out) {
		final float tempy = a.z * b.x - a.x * b.z;
		final float tempz = a.x * b.y - a.y * b.x;
		out.x = a.y * b.z - a.z * b.y;
		out.y = tempy;
		out.z = tempz;
	}
}
