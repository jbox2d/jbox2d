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
 * A 3-by-3 matrix. Stored in column-major order.
 * @author Daniel Murphy
 */
public class Mat33 {
	
	public static final Mat33 IDENTITY = new Mat33(new Vec3(1,0,0), new Vec3(0,1,0), new Vec3(0,0,1));
	
	public final Vec3 col1,col2,col3;
	
	public Mat33(){
		col1 = new Vec3();
		col2 = new Vec3();
		col3 = new Vec3();
	}
	
	public Mat33(Vec3 argCol1, Vec3 argCol2, Vec3 argCol3){
		col1 = argCol1.clone();
		col2 = argCol2.clone();
		col3 = argCol3.clone();
	}
	
	public void setZero(){
		col1.setZero();
		col2.setZero();
		col3.setZero();
	}
	
	/// Multiply a matrix times a vector.
	public static final Vec3 mul( Mat33 A,  Vec3 v){
		return new Vec3(v.x * A.col1.x + v.y * A.col2.x + v.z + A.col3.x,
		                v.x * A.col1.y + v.y * A.col2.y + v.z * A.col3.y,
		                v.x * A.col1.z + v.y * A.col2.z + v.z * A.col3.z);
	}
	
	public static final void mulToOut(Mat33 A, Vec3 v, Vec3 out){
		final float tempy = v.x * A.col1.y + v.y * A.col2.y + v.z * A.col3.y;
		final float tempz = v.x * A.col1.z + v.y * A.col2.z + v.z * A.col3.z;
		out.x = v.x * A.col1.x + v.y * A.col2.x + v.z + A.col3.x;
		out.y = tempy;
		out.z = tempz;
	}
	
	/**
	 * Solve A * x = b, where b is a column vector. This is more efficient
	 * than computing the inverse in one-shot cases.
	 * @param b
	 * @return
	 */
	public final Vec2 solve22(Vec2 b){
		Vec2 x = new Vec2();
		solve22ToOut(b, x);
		return x;
	}
	
	/**
	 * Solve A * x = b, where b is a column vector. This is more efficient
	 * than computing the inverse in one-shot cases.
	 * @param b
	 * @return
	 */
	public final void solve22ToOut(Vec2 b, Vec2 out){
		final float a11 = col1.x, a12 = col2.x, a21 = col1.y, a22 = col2.y;
		float det = a11 * a22 - a12 * a21;
		if (det != 0.0f){
			det = 1.0f / det;
		}
		out.x = det * (a22 * b.x - a12 * b.y);
		out.y = det * (a11 * b.y - a21 * b.x);
	}
	
	// djm pooling from below
	/**
	 * Solve A * x = b, where b is a column vector. This is more efficient
	 * than computing the inverse in one-shot cases.
	 * @param b
	 * @return
	 */
	public final Vec3 solve33(Vec3 b){
		Vec3 x = new Vec3();
		solve33ToOut(b, x);
		return x;
	}
	
	/**
	 * Solve A * x = b, where b is a column vector. This is more efficient
	 * than computing the inverse in one-shot cases.
	 * @param b
	 * @param out the result
	 */
	public final void solve33ToOut(Vec3 b, Vec3 out){
		Vec3.crossToOut(col2, col3, out);
		float det = Vec3.dot(col1, out);
		if (det != 0.0f){
			det = 1.0f / det;
		}
		Vec3.crossToOut(col2, col3, out);
		final float x = det * Vec3.dot(b, out);
		Vec3.crossToOut(b, col3, out);
		final float y = det * Vec3.dot(col1, out);
		Vec3.crossToOut(col2, b, out);
		float z = det * Vec3.dot(col1, out);
		out.x = x;
		out.y = y;
		out.z = z;
	}
}
