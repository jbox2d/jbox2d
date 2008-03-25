
package org.jbox2d.common;


/** A transform contains translation and rotation. It is used to represent
  * the position and orientation of rigid frames.
  */
public class XForm {
	public Vec2 position;
	public Mat22 R;
	
	public static XForm identity;
	
	static{
		identity = new XForm();
		identity.setIdentity();
	}
	
	// The default constructor does nothing (for performance).
	public XForm() {
		position = new Vec2();
		R = new Mat22();
	}

	// Initialize using a position vector and a rotation matrix.
	public XForm(Vec2 _position, Mat22 _R){
		position = _position.clone();
		R = _R.clone();
	}
	
	public void set(XForm xf) {
		position.set(xf.position);
		R.set(xf.R);
	}

	/// Set this to the identity transform.
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
