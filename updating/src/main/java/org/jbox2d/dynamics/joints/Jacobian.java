package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Vec2;

public class Jacobian {
	public final Vec2 linearA = new Vec2();
	public float angularA;
	public final Vec2 linearB = new Vec2();
	public float angularB;
	
	public void setZero(){
		linearA.setZero();
		linearB.setZero();
		angularA = 0f;
		angularB = 0f;
	}
	
	public void set(Vec2 x1, float a1, Vec2 x2, float a2){
		linearA.set(x1);
		linearB.set(x2);
		angularA = a1;
		angularB = a2;
	}
	
	public float compute(Vec2 x1, float a1, Vec2 x2, float a2){
		return Vec2.dot(linearA, x1) + angularA * a1 + Vec2.dot(linearB, x2) + angularB * a2;
	}
}
