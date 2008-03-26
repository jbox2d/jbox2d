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
 * A transform contains translation and rotation. It is used to represent
 * the position and orientation of rigid frames.
 */
public class XForm {
	/** The translation caused by the transform */
	public Vec2 position;
	
	/** A matrix representing a rotation */
	public Mat22 R;
	
	/** The identity transform */
	public static XForm identity;
	
	static{
		identity = new XForm();
		identity.setIdentity();
	}
	
	/** The default constructor. */
	public XForm() {
		position = new Vec2();
		R = new Mat22();
	}
	
	/** Initialize as a copy of another transform. */
	public XForm(XForm xf) {
		position = xf.position.clone();
		R = xf.R.clone();
	}

	/** Initialize using a position vector and a rotation matrix. */
	public XForm(Vec2 _position, Mat22 _R){
		position = _position.clone();
		R = _R.clone();
	}
	
	/** Set this to equal another transform. */
	public void set(XForm xf) {
		position.set(xf.position);
		R.set(xf.R);
	}

	/** Set this to the identity transform. */
	public void setIdentity(){
		position.setZero();
		R.setIdentity();
	}
	
	public static Vec2 mul(XForm T, Vec2 v){
		return T.position.add(T.R.mul(v));
	}

	public static Vec2 mulT(XForm T, Vec2 v){
		return T.R.mulT(v.sub(T.position));
	}
	
	public String toString() {
		String s = "XForm:\n";
		s += "Position: "+position + "\n";
		s += "R: \n"+R+"\n";
		return s;
	}
}
