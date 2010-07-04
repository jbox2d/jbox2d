/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

//#ifndef B2_MATH_H
//#define B2_MATH_H

//#include "b2Settings.h"
//#include <math.h>
//#include <float.h>
//#include <stdlib.h>

final class b2Math{
	//holder for some static methods that were global in C++ version
	
	public static boolean b2IsValid(float x){
		return !(Float.isNAN(x));//_finite(x) != 0;
	}
	
	public static float sqrt(float x){
		return (float)Math.sqrt(x);
	}
	
	public static float sin(float x){
		return (float)Math.sin(x);
	}
	
	public static float cos(float x){
		return (float)Math.cos(x);
	}
	
	public static float32 b2Dot(b2Vec2 a, b2Vec2 b){
		return a.x * b.x + a.y * b.y;
	}

	public static float32 b2Cross(b2Vec2 a, b2Vec2 b){
		return a.x * b.y - a.y * b.x;
	}

	public static b2Vec2 b2Cross(b2Vec2 a, float32 s){
		return new b2Vec2(s*a.y,-s*a.x);
	}

	public static b2Vec2 b2Cross(float32 s, b2Vec2 a){
		return new b2Vec2(-s*a.y,s*a.x);
	}

	public static b2Vec2 b2Mul(b2Mat22 A, b2Vec2 v){
		return new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
	}

	public static b2Vec2 b2MulT(b2Mat22 A, b2Vec2 v){
		return new b2Vec2(b2Dot(v, A.col1), b2Dot(v, A.col2));
	}

	public static b2Vec2 add(b2Vec2 a, b2Vec2 b){
		return new b2Vec2(a.x+b.x,a.y+b.y);
	}

	public static b2Vec2 subtract(b2Vec2 a, b2Vec2 b){
		return new b2Vec2(a.x-b.x,a.y-b.y);
	}

	public static b2Vec2 multiply(float32 s, b2Vec2 a){
		return new b2Vec2(s*a.x,s*a.y);
	}

	public static b2Mat22 add(b2Mat22 A, b2Mat22 B){
		return new b2Mat22( A.col1.x+B.col1.x,
							A.col1.y+B.col1.y,
							A.col2.x+B.col2.x,
							A.col2.y+B.col2.y );
	}

	// A * B
	public static b2Mat22 b2Mul(b2Mat22 A, b2Mat22 B){
		return new b2Mat22( b2Mul(A, B.col1) , b2Mul(A, B.col2) );
	}

	// A^T * B
	public static b2Mat22 b2MulT(b2Mat22 A, b2Mat22 B){
//		b2Vec2 c1 = new b2Vec2(b2Dot(A.col1, B.col1), b2Dot(A.col2, B.col1));
//		b2Vec2 c2 = new b2Vec2(b2Dot(A.col1, B.col2), b2Dot(A.col2, B.col2));
//		return new b2Mat22(c1,c2);
		return new b2Mat22(b2Dot(A.col1, B.col1), b2Dot(A.col2, B.col1),
						   b2Dot(A.col1, B.col2), b2Dot(A.col2, B.col2));
	}

	public static float32 b2Abs(float32 a){
		return a > 0.0f ? a : -a;
	}

	public static b2Vec2 b2Abs(b2Vec2 a){
		return new b2Vec2( a.x>0.0f?a.x:-a.x , a.y>0.0f?a.y:-a.y );
	}

	public static b2Mat22 b2Abs(b2Mat22 A){
		return new b2Mat22(b2Abs(A.col1.x),b2Abs(A.col1.y),
						   b2Abs(A.col2.x),b2Abs(A.col2.y));
	}
	
	
	// b2Random number in range [-1,1]
	public static float32 b2Random(){
	/*	float32 r = (float32)rand();
		r /= RAND_MAX;
		r = 2.0f * r - 1.0f;
		return r;*/
	//TODO: insert decent Java implementation of random generator
	}

	public static float32 b2Random(float32 lo, float32 hi){
	/*	float32 r = (float32)rand();
		r /= RAND_MAX;
		r = (hi - lo) * r + lo;
		return r;*/
	//TODO: insert decent Java implementation of random generator
	}

	
	// "Next Largest Power of 2
	// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
	// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
	// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
	// largest power of 2. For a 32-bit value:"
	public static uint32 b2NextPowerOfTwo(uint32 x){
		x |= (x >> 1);
		x |= (x >> 2);
		x |= (x >> 4);
		x |= (x >> 8);
		x |= (x >> 16);
		return x + 1;
	}

	public static boolean b2IsPowerOfTwo(uint32 x){
		boolean result = x > 0 && (x & (x - 1)) == 0;
		return result;
	}

	public static b2Vec2 b2Min(b2Vec2 a, b2Vec2 b){
		return new b2Vec2( b2Min(a.x,b.x), b2Min(a.y,b.y) );
	}
	
	public static b2Vec2 b2Max(b2Vec2 a, b2Vec2 b){
		return new b2Vec2( b2Max(a.x,b.x), b2Max(a.y,b.y) );
	}
	
	public static b2Vec2 b2Clamp(b2Vec2 a, b2Vec2 low, b2Vec2 high){
		return b2Max(low, b2Min(a, high));
	}
	
	//Following four functions were template functions in C++
	//Avoiding this in Java for the moment, so we'll write them as needed...
	public static float32 b2Min(float32 a, float32 b){
		return a < b ? a : b;
	}
	
	public static float32 b2Max(float32 a, float32 b){
		return a > b ? a : b;
	}
	
	public static float32 b2Clamp(float32 a, float32 low, float32 high){
		return b2Max(low, b2Min(a, high));
	}

	//b2Swap disabled for the moment in Java version - we may need to
	//add specialized versions once I see where this is actually used
	//in the code.  May be worth just inlining by hand, anyways...
	/*
	public static void b2Swap(T& a, T& b){
		T tmp = a;
		a = b;
		b = tmp;
	}*/
	
}

//Marked final to allow for inlining
final class b2Vec2{
	public b2Vec2() {}
	public b2Vec2(float32 _x, float32 _y){
		x = _x; y = _y;
	}
	
	public void SetZero() { x = 0.0f; y = 0.0f; }
	public void Set(float32 x_, float32 y_) { x = x_; y = y_; }
	public void Set(b2Vec2 v){ x = v.x; y = v.y; }

	public b2Vec2 negative() { b2Vec2 v; v.Set(-x, -y); return v; }
	
	public static b2Vec2 Make(float32 x_, float32 y_){
		return new b2Vec(x_,y_);
		//v.Set(x_, y_);
		//return v;
	}

	public void add(b2Vec2 v){
		x += v.x; y += v.y;
	}
	
	public void subtract(b2Vec2 v){
		x -= v.x; y -= v.y;
	}

	public void multiply(float32 a){
		x *= a; y *= a;
	}

	public float32 Length(){
		return b2Math.sqrt(x * x + y * y);
	}

	public float32 Normalize(){
		float32 length = Length();
		if (length < FLT_EPSILON){
			return 0.0f;
		}
		float32 invLength = 1.0f / length;
		x *= invLength;
		y *= invLength;
		return length;
	}

	public boolean IsValid(){
		return b2Math.b2IsValid(x) && b2Math.b2IsValid(y);
	}

	public float32 x, y;
}


final class b2Mat22{
	public b2Mat22() {}
	public b2Mat22(b2Vec2 c1, b2Vec2 c2){
		//Java note: in C++ version, c1 and c2 are marked as const
		//so they should not change - hence we copy them here.
		col1 = new b2Vec2(c1.x,c1.y);
		col2 = new b2Vec2(c2.x,c2.y);
	}

	public b2Mat22(float32 angle){
		float32 c = b2Math.cos(angle), s = b2Math.sin(angle);
		col1 = new b2Vec2(c,-s);
		col2 = new b2Vec2(s,c);
	}

	public void Set(b2Vec2 c1, b2Vec2 c2){
		col1 = new b2Vec2(c1.x,c1.y);
		col2 = new b2Vec2(c2.x,c2.y);
	}

	public void Set(float32 angle){
		float32 c = b2Math.cos(angle), s = b2Math.sin(angle);
		col1.x = c; col2.x = -s;
		col1.y = s; col2.y = c;
	}

	public b2Mat22 Invert(){
		float32 a = col1.x, b = col2.x, c = col1.y, d = col2.y;
		float32 det = a * d - b * c;
		b2Assert(det != 0.0f);
		det = 1.0f / det;
//		b2Mat22 B;
//		B.col1.x =  det * d;	B.col2.x = -det * b;
//		B.col1.y = -det * c;	B.col2.y =  det * a;
		return new b2Mat22(det*d,-det*c,-det*b,det*a);
	}
	
	public b2Mat22(float c1x,float c1y, float c2x, float c2y){
		col1 = new b2Vec2(c1x,c1y);
		col2 = new b2Vec2(c2x,c2y);
	}

	public b2Vec2 col1, col2;
}

//#endif
