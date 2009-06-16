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

/**
 * A 2x2 matrix class.
 * djm: added ToOut methods, optimized, and added modifiers
 * @author ewjordan
 *
 */
public class Mat22 {
	public Vec2 col1, col2;

	/** Convert the matrix to printable format. */
	@Override
	public String toString() {
		String s = "";
		s += "["+col1.x+","+col2.x+"]\n";
		s += "["+col1.y+","+col2.y+"]";
		return s;
	}

	/** Construct zero matrix.  Note: this is NOT an identity matrix!
	 * djm fixed double allocation problem*/
	public Mat22() {
		col1 = new Vec2();
		col2 = new Vec2();
	}

	/**
	 * Create a matrix representing a rotation.
	 * @param angle Rotation (in radians) that matrix represents.
	 */
	public Mat22(float angle) {
		this();
		setAngle(angle);
	}

	/**
	 * Create a matrix with given vectors as columns.
	 * @param c1 Column 1 of matrix
	 * @param c2 Column 2 of matrix
	 */
	public Mat22(Vec2 c1, Vec2 c2) {
		col1 = c1.clone();
		col2 = c2.clone();
	}

	/**
	 * Create a matrix from four floats.
	 * @param col1x
	 * @param col2x
	 * @param col1y
	 * @param col2y
	 */
	public Mat22(float col1x, float col2x, float col1y, float col2y) {
		col1 = new Vec2(col1x, col1y);
		col2 = new Vec2(col2x, col2y);
	}

	/**
	 * Set as a copy of another matrix.
	 * @param m Matrix to copy
	 */
	public final void set(Mat22 m) {
		col1.x = m.col1.x;
		col1.y = m.col1.y;
		col2.x = m.col2.x;
		col2.y = m.col2.y;
	}
	
	public final void set(float col1x, float col2x, float col1y, float col2y) {
		col1.x = col1x;
		col1.y = col1y;
		col2.x = col2x;
		col2.y = col2y;
	}

	/**
	 * Return a clone of this matrix.
	 * djm fixed double allocation
	 */
	@Override
	public final Mat22 clone() {
		return new Mat22(col1, col2);
	}

	/**
	 * Set as a matrix representing a rotation.
	 * @param angle Rotation (in radians) that matrix represents.
	 */
	public final void set(float angle) {
		float c = (float)Math.cos(angle), s = (float)Math.sin(angle);
		col1.x = c; col2.x = -s;
		col1.y = s; col2.y = c;
	}

	/**
	 * Set as the identity matrix.
	 */
	public final void setIdentity() {
		col1.x = 1.0f;
		col2.x = 0.0f;
		col1.y = 0.0f;
		col2.y = 1.0f;
	}

	/**
	 * Set as the zero matrix.
	 */
	public final void setZero() {
		col1.x = 0.0f;
		col2.x = 0.0f;
		col1.y = 0.0f;
		col2.y = 0.0f;
	}

	/**
	 * Set as a matrix representing a rotation.
	 * @param angle Rotation (in radians) that matrix represents.
	 */
	public final void setAngle(float angle) {
		float c = (float) Math.cos(angle);
		float s = (float) Math.sin(angle);
		col1.x = c;
		col2.x = -s;
		col1.y = s;
		col2.y = c;
	}

	/**
	 * Set by column vectors.
	 * @param c1 Column 1
	 * @param c2 Column 2
	 */
	public final void set(Vec2 c1, Vec2 c2) {
		col1.x = c1.x;
		col2.x = c2.x;
		col1.y = c1.y;
		col2.y = c2.y;
	}

	/** Returns the inverted Mat22 - does NOT invert the matrix locally! */
	public final Mat22 invert() {
		float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
		Mat22 B = new Mat22();
		float det = a * d - b * c;
		// b2Assert(det != 0.0f);
		det = 1.0f / det;
		B.col1.x = det * d;
		B.col2.x = -det * b;
		B.col1.y = -det * c;
		B.col2.y = det * a;
		return B;
	}

	public final void invertToOut(Mat22 out){
		float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
		float det = a * d - b * c;
		// b2Assert(det != 0.0f);
		det = 1.0f / det;
		out.col1.x = det * d;
		out.col2.x = -det * b;
		out.col1.y = -det * c;
		out.col2.y = det * a;
	}

	/**
	 * Return the matrix composed of the absolute values of all elements.
	 * djm: fixed double allocation
	 * @return Absolute value matrix
	 */
	public final Mat22 abs() {
		return new Mat22(Math.abs(col1.x),
		                 Math.abs(col2.x),
		                 Math.abs(col1.y),
		                 Math.abs(col2.y));
	}

	/* djm: added */
	public final void absLocal(){
		col1.absLocal();
		col2.absLocal();
	}

	/**
	 * Return the matrix composed of the absolute values of all elements.
	 * @return Absolute value matrix
	 */
	public final static Mat22 abs(Mat22 R) {
		return R.abs();
	}

	/* djm created */
	public static void absToOut(Mat22 R, Mat22 out){
		out.col1.x = Math.abs(R.col1.x);
		out.col1.y = Math.abs(R.col1.y);
		out.col2.x = Math.abs(R.col2.x);
		out.col2.y = Math.abs(R.col2.y);
	}

	/**
	 * Multiply a vector by this matrix.
	 * @param v Vector to multiply by matrix.
	 * @return Resulting vector
	 */
	public final Vec2 mul(Vec2 v) {
		return new Vec2(col1.x * v.x + col2.x * v.y, col1.y * v.x + col2.y
		                * v.y);
	}

	/* djm added */
	public final void mulToOut(Vec2 v, Vec2 out){
		float tempy = col1.y * v.x + col2.y * v.y;
		out.x = col1.x * v.x + col2.x * v.y;
		out.y = tempy;
	}


	/**
	 * Multiply another matrix by this one (this one on left).
	 * djm optimized
	 * @param R
	 * @return
	 */
	public final Mat22 mul(Mat22 R) {
		/*Mat22 C = new Mat22();
		 *C.set(this.mul(R.col1), this.mul(R.col2));
		 *return C;
		 */
		Mat22 C = new Mat22();
		C.col1.x = col1.x * R.col1.x + col2.x * R.col1.y;
		C.col1.y = col1.y * R.col1.x + col2.y * R.col1.y;
		C.col2.x = col1.x * R.col2.x + col2.x * R.col2.y;
		C.col2.y = col1.y * R.col2.x + col2.y * R.col2.y;
		//C.set(col1,col2);
		return C;
	}

	/* djm: created */
	public final void mulToOut(Mat22 R, Mat22 out){
		this.mulToOut( R.col1, out.col1);
		this.mulToOut( R.col2, out.col2);
	}

	/**
	 * Multiply another matrix by the transpose of this one (transpose of this one on left).
	 * djm: optimized
	 * @param B
	 * @return
	 */
	public final Mat22 mulTrans(Mat22 B) {
		/*
		 * Vec2 c1 = new Vec2(Vec2.dot(this.col1, B.col1), Vec2.dot(this.col2,
                B.col1));
        Vec2 c2 = new Vec2(Vec2.dot(this.col1, B.col2), Vec2.dot(this.col2,
                B.col2));
        Mat22 C = new Mat22();
        C.set(c1, c2);
        return C;
		 */
		Mat22 C = new Mat22();
		
		C.col1.x = Vec2.dot(this.col1, B.col1);
		C.col1.y = Vec2.dot(this.col2, B.col1);

		C.col2.x = Vec2.dot(this.col1, B.col2);
		C.col2.y = Vec2.dot(this.col2, B.col2);
		return C;
	}

	/* djm added */
	public final void mulTransToOut(Mat22 B, Mat22 out){
		//Vec2 c1 = Stack.alloc( Vec2.class);
		out.col1.x = Vec2.dot(this.col1, B.col1);
		out.col1.y = Vec2.dot(this.col2, B.col1);

		//Vec2 c2 = Stack.alloc( Vec2.class);
		out.col2.x = Vec2.dot(this.col1, B.col2);
		out.col2.y = Vec2.dot(this.col2, B.col2);

		//out.set(c1,c2);
	}

	/**
	 * Multiply a vector by the transpose of this matrix.
	 * @param v
	 * @return
	 */
	public final Vec2 mulTrans(Vec2 v) {
		return new Vec2(Vec2.dot(v, col1), Vec2.dot(v, col2));
	}

	/* djm added */
	public final void mulTransToOut(Vec2 v, Vec2 out){
		out.x = Vec2.dot(v, col1);
		out.y = Vec2.dot(v, col2);
	}

	/**
	 * Add this matrix to B, return the result.
	 * @param B
	 * @return
	 */
	public final Mat22 add(Mat22 B) {
		return new Mat22(col1.add(B.col1), col2.add(B.col2));
	}

	/**
	 * Add B to this matrix locally.
	 * @param B
	 * @return
	 */
	public final Mat22 addLocal(Mat22 B) {
		col1.addLocal(B.col1);
		col2.addLocal(B.col2);
		return this;
	}

	/**
	 * Solve A * x = b where A = this matrix.
	 * @return The vector x that solves the above equation.
	 */
	public final Vec2 solve(Vec2 b) {
		float a11 = col1.x, a12 = col2.x, a21 = col1.y, a22 = col2.y;
		float det = a11 * a22 - a12 * a21;
		assert(det != 0.0f);
		det = 1.0f / det;
		Vec2 x = new Vec2( det * (a22 * b.x - a12 * b.y),
		                         det * (a11 * b.y - a21 * b.x) );
		return x;
	}

	/* djm added */
	public final void solveToOut(Vec2 b, Vec2 out) {
		float a11 = col1.x, a12 = col2.x, a21 = col1.y, a22 = col2.y;
		float det = a11 * a22 - a12 * a21;
		assert(det != 0.0f);
		det = 1.0f / det;
		float tempx = det * (a22 * b.x - a12 * b.y);
		float tempy =  det * (a11 * b.y - a21 * b.x) ;
		out.x = tempx;
		out.y = tempy;
	}

	public final static Vec2 mul(Mat22 R, Vec2 v) {
		return R.mul(v);
	}

	/* djm added */
	public final static void mulToOut(Mat22 R, Vec2 v, Vec2 out) {
		R.mulToOut(v, out);
	}

	public final static Mat22 mul(Mat22 A, Mat22 B){
		return A.mul(B);
	}

	/* djm added */
	public final static void mulToOut(Mat22 A, Mat22 B, Mat22 out){
		A.mulToOut( B, out);
	}

	public final static Vec2 mulTrans(Mat22 R, Vec2 v) {
		return R.mulTrans(v);
	}

	/* djm added */
	public final static void mulTransToOut(Mat22 R, Vec2 v, Vec2 out) {
		R.mulTransToOut(v, out);
	}

	public final static Mat22 mulTrans(Mat22 A, Mat22 B){
		return A.mulTrans(B);
	}

	/* djm added */
	public final static void mulTransToOut(Mat22 A, Mat22 B, Mat22 out){
		A.mulTransToOut(B, out);
	}
}
