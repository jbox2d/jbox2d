/**
 * Created at 3:26:14 AM Jan 11, 2011
 */
package org.jbox2d.pooling;

import java.util.HashMap;
import java.util.Stack;

import org.jbox2d.collision.AABB;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.Vec3;

/**
 * Provides object pooling for all objects used in the engine.  Objects
 * retrieved from here should only be used temporarily, and then pushed back (with
 * the exception of arrays).
 * @author Daniel Murphy
 */
public class WorldPool {
	
	private final Stack<Vec2> vecs = new Stack<Vec2>();
	private final Stack<Vec3> vec3s = new Stack<Vec3>();
	private final Stack<Mat22> mats = new Stack<Mat22>();
	private final Stack<AABB> aabbs = new Stack<AABB>();
	
	private final HashMap<Integer, float[]> afloats = new HashMap<Integer, float[]>();
	private final HashMap<Integer, int[]> aints = new HashMap<Integer, int[]>();
	private final HashMap<Integer, Vec2[]> avecs = new HashMap<Integer, Vec2[]>();
	
	public final Stack<Vec2> getVec2Stack(){
		return vecs;
	}
	
	public final Vec2 popVec2(){
		if(!Settings.POOLING){
			return new Vec2();
		}
		if(vecs.isEmpty()){
			vecs.push(new Vec2());
			vecs.push(new Vec2());
			vecs.push(new Vec2());
			vecs.push(new Vec2());
			vecs.push(new Vec2());
			return new Vec2();
		}
		return vecs.pop();
	}
	
	public final void pushVec2(Vec2 argVec2){
		if(!Settings.POOLING){
			return;
		}
		vecs.push(argVec2);
	}
	
	public final void pushVec2(Vec2... argVec2s){
		if(!Settings.POOLING){
			return;
		}
		for(Vec2 v : argVec2s){
			vecs.push(v);
		}
	}
	
	public final Stack<Vec3> getVec3Stack(){
		return vec3s;
	}
	
	public final Vec3 popVec3(){
		if(!Settings.POOLING){
			return new Vec3();
		}
		if(vec3s.isEmpty()){
			vec3s.push(new Vec3());
			vec3s.push(new Vec3());
			vec3s.push(new Vec3());
			vec3s.push(new Vec3());
			vec3s.push(new Vec3());
			return new Vec3();
		}
		return vec3s.pop();
	}
	
	public final void pushVec3(Vec3 argVec3){
		if(!Settings.POOLING){
			return;
		}
		vec3s.push(argVec3);
	}
	
	public final void pushVec3(Vec3... argVec3s){
		if(!Settings.POOLING){
			return;
		}
		for(Vec3 v : argVec3s){
			vec3s.push(v);
		}
	}
	
	
	public final Stack<Mat22> getMat22Stack(){
		return mats;
	}
	
	public final Mat22 popMat22(){
		if(!Settings.POOLING){
			return new Mat22();
		}
		if(mats.isEmpty()){
			mats.push(new Mat22());
			mats.push(new Mat22());
			mats.push(new Mat22());
			mats.push(new Mat22());
			mats.push(new Mat22());
			return new Mat22();
		}
		return mats.pop();
	}
	
	public final void pushMat22(Mat22 argMat){
		if(!Settings.POOLING){
			return;
		}
		mats.push(argMat);
	}
	
	public final void pushMat22(Mat22... argMats){
		if(!Settings.POOLING){
			return;
		}
		for(Mat22 m : argMats){
			mats.push(m);
		}
	}
	
	public final Stack<AABB> getAABBStack(){
		return aabbs;
	}
	
	public final AABB popAABB(){
		if(!Settings.POOLING){
			return new AABB();
		}
		if(aabbs.isEmpty()){
			aabbs.push(new AABB());
			aabbs.push(new AABB());
			aabbs.push(new AABB());
			aabbs.push(new AABB());
			aabbs.push(new AABB());
			return new AABB();
		}
		return aabbs.pop();
	}
	
	public final void pushAABB(AABB argAABB){
		aabbs.push(argAABB);
	}
	
	
	public final float[] getFloatArray(int argLength){
		if(!afloats.containsKey(argLength)){
			afloats.put(argLength, new float[argLength]);
		}
		
		assert(afloats.get(argLength).length == argLength) : "Array not built with correct length";
		return afloats.get(argLength);
	}
	
	public final int[] getIntArray(int argLength){
		if(!aints.containsKey(argLength)){
			aints.put(argLength, new int[argLength]);
		}
		
		assert(aints.get(argLength).length == argLength) : "Array not built with correct length";
		return aints.get(argLength);
	}
	
	public final Vec2[] getVec2Array(int argLength){
		if(!avecs.containsKey(argLength)){
			Vec2[] ray = new Vec2[argLength];
			for(int i=0; i<argLength; i++){
				ray[i] = new Vec2();
			}
			avecs.put(argLength, ray);
		}
		
		assert(avecs.get(argLength).length == argLength) : "Array not built with correct length";
		return avecs.get(argLength);
	}
}
