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
import org.jbox2d.pooling.PoolingStack.PoolContainer;

/**
 * Provides object pooling for all objects used in the engine.  Objects
 * retrieved from here should only be used temporarily, and then pushed back (with
 * the exception of arrays).
 * @author Daniel Murphy
 */
public class WorldPool {
	
	private final PoolingStack<Vec2> vecs = new PoolingStack<Vec2>(Vec2.class, 100);
	private final PoolingStack<Vec3> vec3s = new PoolingStack<Vec3>(Vec3.class, 100);
	private final PoolingStack<Mat22> mats = new PoolingStack<Mat22>(Mat22.class, 100);
	private final PoolingStack<AABB> aabbs = new PoolingStack<AABB>(AABB.class, 100);
	
	private final HashMap<Integer, float[]> afloats = new HashMap<Integer, float[]>();
	private final HashMap<Integer, int[]> aints = new HashMap<Integer, int[]>();
	private final HashMap<Integer, Vec2[]> avecs = new HashMap<Integer, Vec2[]>();
	
	public final PoolingStack<Vec2> getVec2Stack(){
		return vecs;
	}
	
	/**
	 * @return
	 * @see org.jbox2d.pooling.PoolingStack#get()
	 */
	public final Vec2 popVec2() {
		if(!Settings.POOLING){
			return new Vec2();
		}
		return vecs.pop();
	}

	/**
	 * @param argNum
	 * @return
	 * @see org.jbox2d.pooling.PoolingStack#get(int)
	 */
	public final PoolContainer<Vec2> popVec2(int argNum) {
		if(!Settings.POOLING){
			PoolContainer<Vec2> pc = new PoolContainer<Vec2>();
			Vec2[] ray = new Vec2[PoolContainer.MAX_MEMBERS];
			for(int i=0; i<argNum; i++){
				ray[i] = new Vec2();
			}
			pc.populate(ray);
			return pc;
		}
		return vecs.pop(argNum);
	}

	/**
	 * @param argNum
	 * @see org.jbox2d.pooling.PoolingStack#reclaim(int)
	 */
	public final void pushVec2(int argNum) {
		if(!Settings.POOLING){
			return;
		}
		vecs.push(argNum);
	}

	public final PoolingStack<Vec3> getVec3Stack(){
		return vec3s;
	}
	
	public final Vec3 popVec3(){
		if(!Settings.POOLING){
			return new Vec3();
		}
		return vec3s.pop();
	}
	
	public final PoolContainer<Vec3> popVec3(int argNum){
		if(!Settings.POOLING){
			PoolContainer<Vec3> pc = new PoolContainer<Vec3>();
			Vec3[] ray = new Vec3[PoolContainer.MAX_MEMBERS];
			for(int i=0; i<argNum; i++){
				ray[i] = new Vec3();
			}
			pc.populate(ray);
			return pc;
		}
		return vec3s.pop(argNum);
	}
	
	public final void pushVec3(int argNum){
		if(!Settings.POOLING){
			return;
		}
		vec3s.push(argNum);
	}
	
	
	public final PoolingStack<Mat22> getMat22Stack(){
		return mats;
	}
	
	public final Mat22 popMat22(){
		if(!Settings.POOLING){
			return new Mat22();
		}
		return mats.pop();
	}
	
	public final PoolContainer<Mat22> popMat22(int argNum){
		if(!Settings.POOLING){
			PoolContainer<Mat22> pc = new PoolContainer<Mat22>();
			Mat22[] ray = new Mat22[PoolContainer.MAX_MEMBERS];
			for(int i=0; i<argNum; i++){
				ray[i] = new Mat22();
			}
			pc.populate(ray);
			return pc;
		}
		return mats.pop(argNum);
	}
	
	public final void pushMat22(int argNum){
		if(!Settings.POOLING){
			return;
		}
		mats.push(argNum);
	}
	
	
	public final PoolingStack<AABB> getAABBStack(){
		return aabbs;
	}
	
	public final AABB popAABB(){
		if(!Settings.POOLING){
			return new AABB();
		}
		return aabbs.pop();
	}
	
	public final PoolContainer<AABB> popAABB(int argNum){
		if(!Settings.POOLING){
			PoolContainer<AABB> pc = new PoolContainer<AABB>();
			AABB[] ray = new AABB[PoolContainer.MAX_MEMBERS];
			for(int i=0; i<argNum; i++){
				ray[i] = new AABB();
			}
			pc.populate(ray);
			return pc;
		}
		return aabbs.pop(argNum);
	}
	
	public final void pushAABB(int argNum){
		if(!Settings.POOLING){
			return;
		}
		aabbs.push(argNum);
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
