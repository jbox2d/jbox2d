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

// djm created from build 218
public class Mat33 {
	public Vec3 col1,col2,col3;
	
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
	
	//djm pooled
	private Vec3 temp = new Vec3();
	public Vec3 solve(Vec3 b) {
		Vec3.crossToOut( col1, col2, temp);
		float det = Vec3.dot(col1, temp);
		assert(det != 0.0f);
		det = 1.0f / det;
		Vec3 x = new Vec3();
		Vec3.crossToOut( col2, col3, temp);
		x.x = det * Vec3.dot(b, temp);
		Vec3.crossToOut( b, col3, temp);
		x.y = det * Vec3.dot(col1, temp);
		Vec3.crossToOut( col2, b, temp);
		x.z = det * Vec3.dot(col1, temp);
		return x;
	}
	
	public void solveToOut(Vec3 b, Vec3 out){
		Vec3.crossToOut( col1, col2, out);
		float det = Vec3.dot(col1, out);
		assert(det != 0.0f);
		det = 1.0f / det;
		Vec3.crossToOut( col2, col3, out);
		final float tempx = det * Vec3.dot(b, out);
		Vec3.crossToOut( b, col3, out);
		final float tempy = det * Vec3.dot(col1, out);
		Vec3.crossToOut( col2, b, out);
		final float tempz = det * Vec3.dot(col1, out);
		out.x = tempx;
		out.y = tempy;
		out.z = tempz;
	}
}
