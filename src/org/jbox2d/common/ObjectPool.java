package org.jbox2d.common;

import java.util.Stack;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.BoundValues;
import org.jbox2d.collision.Distance;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.MassData;
import org.jbox2d.collision.shapes.CollideCircle;
import org.jbox2d.collision.shapes.CollidePoly;
import org.jbox2d.dynamics.Island;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.dynamics.contacts.ContactPoint;
import org.jbox2d.dynamics.contacts.ContactSolver;

public final class ObjectPool {
	
	private static final ThreadLocal<LargePool> largePool = new ThreadLocal<LargePool>(){
		protected LargePool initialValue() {
			return new LargePool();
		}
	};
	
	private static final class LargePool {
		private final Stack<Vec2> vec2Pool = new Stack<Vec2>();
		private final Stack<Mat22> mat22Pool = new Stack<Mat22>();
		private final Stack<XForm> xFormPool = new Stack<XForm>();
		private final Stack<Sweep> sweepPool = new Stack<Sweep>();
		private final Stack<AABB> aabbPool = new Stack<AABB>();
		private final Stack<MassData> massPool = new Stack<MassData>();
		private final Stack<Manifold> manifoldPool = new Stack<Manifold>();
		private final Stack<ContactPoint> contactPointPool = new Stack<ContactPoint>();
		private final Stack<RaycastResult> raycastResultPool = new Stack<RaycastResult>();
		private final Stack<BoundValues> boundValuesPool = new Stack<BoundValues>();
		private final Stack<TimeStep> timeStepPool = new Stack<TimeStep>();
		private final Stack<Integer[]> twoIntStack = new Stack<Integer[]>();
		private final Stack<Island> islandPool = new Stack<Island>();
		private final Stack<ContactSolver> contactSolverPool = new Stack<ContactSolver>();
		
		private final Distance distances = new Distance();
		private final CollideCircle collideCircles = new CollideCircle();
		private final CollidePoly collidePolys = new CollidePoly();
	}

	
	private ObjectPool() {};
	
	public static final Integer[] getTwoInts(){
		Stack<Integer[]> pool = largePool.get().twoIntStack;
		
		if(pool.isEmpty()){
			Integer[] ints = {0,0};
			pool.push(ints);
			Integer[] ints2 = {0,0};
			pool.push(ints2);
			Integer[] ints3 = {0,0};
			pool.push(ints3);
			Integer[] ints4 = {0,0};
			pool.push(ints4);
			Integer[] ints5 = {0,0};
			pool.push(ints5);
		}
		
		return pool.pop();
	}
	
	public static final void returnTwoInts(Integer[] argToRecycle){
		Stack<Integer[]> pool = largePool.get().twoIntStack;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		assert (argToRecycle.length == 2) : "Array has to be of 2 integers";
		pool.push(argToRecycle);
	}

	public static final Vec2 getVec2() {
		Stack<Vec2> pool = largePool.get().vec2Pool;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new Vec2());
			pool.push(new Vec2());
			pool.push(new Vec2());
			pool.push(new Vec2());
			pool.push(new Vec2());
		}

		return pool.pop();
	}

	public static final Vec2 getVec2(Vec2 toCopy) {
		Stack<Vec2> pool = largePool.get().vec2Pool;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new Vec2());
			pool.push(new Vec2());
			pool.push(new Vec2());
			pool.push(new Vec2());
			pool.push(new Vec2());
		}

		return pool.pop().set(toCopy);
	}

	public static final void returnVec2(Vec2 argToRecycle) {
		Stack<Vec2> pool = largePool.get().vec2Pool;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}

	public static final Mat22 getMat22() {
		Stack<Mat22> pool = largePool.get().mat22Pool;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new Mat22());
			pool.push(new Mat22());
			pool.push(new Mat22());
			pool.push(new Mat22());
			pool.push(new Mat22());
		}

		return pool.pop();
	}

	public static final Mat22 getMat22(Mat22 toCopy) {
		Stack<Mat22> pool = largePool.get().mat22Pool;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new Mat22());
			pool.push(new Mat22());
			pool.push(new Mat22());
			pool.push(new Mat22());
			pool.push(new Mat22());
		}

		return pool.pop().set(toCopy);
	}

	public static final void returnMat22(Mat22 argToRecycle) {
		Stack<Mat22> pool =  largePool.get().mat22Pool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}

	public static final XForm getXForm() {
		Stack<XForm> pool =  largePool.get().xFormPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new XForm());
			pool.push(new XForm());
			pool.push(new XForm());
			pool.push(new XForm());
			pool.push(new XForm());
		}

		return pool.pop();
	}

	public static final XForm getXForm(XForm toCopy) {
		Stack<XForm> pool =  largePool.get().xFormPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new XForm());
			pool.push(new XForm());
			pool.push(new XForm());
			pool.push(new XForm());
			pool.push(new XForm());
		}

		return pool.pop().set(toCopy);
	}

	public static final void returnXForm(XForm argToRecycle) {
		Stack<XForm> pool =  largePool.get().xFormPool;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final Sweep getSweep() {
		Stack<Sweep> pool =  largePool.get().sweepPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new Sweep());
			pool.push(new Sweep());
			pool.push(new Sweep());
			pool.push(new Sweep());
			pool.push(new Sweep());
		}

		return pool.pop();
	}

	public static final Sweep getSweep(Sweep toCopy) {
		Stack<Sweep> pool =  largePool.get().sweepPool;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new Sweep());
			pool.push(new Sweep());
			pool.push(new Sweep());
			pool.push(new Sweep());
			pool.push(new Sweep());
		}

		return pool.pop().set(toCopy);
	}

	public static final void returnSweep(Sweep argToRecycle) {
		Stack<Sweep> pool =  largePool.get().sweepPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final AABB getAABB() {
		Stack<AABB> pool =  largePool.get().aabbPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new AABB());
			pool.push(new AABB());
			pool.push(new AABB());
			pool.push(new AABB());
			pool.push(new AABB());
		}

		return pool.pop();
	}

	public static final AABB getAABB(AABB toCopy) {
		Stack<AABB> pool =  largePool.get().aabbPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new AABB());
			pool.push(new AABB());
			pool.push(new AABB());
			pool.push(new AABB());
			pool.push(new AABB());
		}

		return pool.pop().set(toCopy);
	}

	public static final void returnAABB(AABB argToRecycle) {
		Stack<AABB> pool =  largePool.get().aabbPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final MassData getMassData() {
		Stack<MassData> pool =  largePool.get().massPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new MassData());
			pool.push(new MassData());
			pool.push(new MassData());
			pool.push(new MassData());
			pool.push(new MassData());
		}

		return pool.pop();
	}

	public static final MassData getMassData(MassData toCopy) {
		Stack<MassData> pool =  largePool.get().massPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new MassData());
			pool.push(new MassData());
			pool.push(new MassData());
			pool.push(new MassData());
			pool.push(new MassData());
		}

		return pool.pop().set(toCopy);
	}

	public static final void returnMassData(MassData argToRecycle) {
		Stack<MassData> pool =  largePool.get().massPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}

	public static final Manifold getManifold() {
		Stack<Manifold> pool =  largePool.get().manifoldPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new Manifold());
			pool.push(new Manifold());
			pool.push(new Manifold());
			pool.push(new Manifold());
			pool.push(new Manifold());
		}

		return pool.pop();
	}

	public static final Manifold getManifold(Manifold toCopy) {
		Stack<Manifold> pool =  largePool.get().manifoldPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new Manifold());
			pool.push(new Manifold());
			pool.push(new Manifold());
			pool.push(new Manifold());
			pool.push(new Manifold());
		}

		return pool.pop().set(toCopy);
	}

	public static final void returnManifold(Manifold argToRecycle) {
		Stack<Manifold> pool =  largePool.get().manifoldPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final ContactPoint getContactPoint() {
		Stack<ContactPoint> pool =  largePool.get().contactPointPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new ContactPoint());
			pool.push(new ContactPoint());
			pool.push(new ContactPoint());
			pool.push(new ContactPoint());
			pool.push(new ContactPoint());
		}

		return pool.pop();
	}

	public static final void returnContactPoint(ContactPoint argToRecycle) {
		Stack<ContactPoint> pool =  largePool.get().contactPointPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final RaycastResult getRaycastResult() {
		Stack<RaycastResult> pool =  largePool.get().raycastResultPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new RaycastResult());
			pool.push(new RaycastResult());
			pool.push(new RaycastResult());
			pool.push(new RaycastResult());
			pool.push(new RaycastResult());
		}

		return pool.pop();
	}

	public static final RaycastResult getRaycastResult(RaycastResult toCopy) {
		Stack<RaycastResult> pool =  largePool.get().raycastResultPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new RaycastResult());
			pool.push(new RaycastResult());
			pool.push(new RaycastResult());
			pool.push(new RaycastResult());
			pool.push(new RaycastResult());
		}

		return pool.pop().set(toCopy);
	}

	public static final void returnRaycastResult(RaycastResult argToRecycle) {
		Stack<RaycastResult> pool =  largePool.get().raycastResultPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final BoundValues getBoundValues() {
		Stack<BoundValues> pool =  largePool.get().boundValuesPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new BoundValues());
			pool.push(new BoundValues());
			pool.push(new BoundValues());
			pool.push(new BoundValues());
			pool.push(new BoundValues());
		}

		return pool.pop();
	}

	public static final void returnBoundValues(BoundValues argToRecycle) {
		Stack<BoundValues> pool =  largePool.get().boundValuesPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final TimeStep getTimeStep() {
		Stack<TimeStep> pool =  largePool.get().timeStepPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

		if (pool.isEmpty()) {
			pool.push(new TimeStep());
			pool.push(new TimeStep());
			pool.push(new TimeStep());
			pool.push(new TimeStep());
			pool.push(new TimeStep());
		}

		return pool.pop();
	}

	public static final void returnTimeStep(TimeStep argToRecycle) {
		Stack<TimeStep> pool =  largePool.get().timeStepPool ;
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final Island getIsland() {
		Stack<Island> pool =  largePool.get().islandPool ;

		if (pool.isEmpty()) {
			pool.push(new Island());
			pool.push(new Island());
			pool.push(new Island());
			pool.push(new Island());
			pool.push(new Island());
		}

		return pool.pop();
	}

	public static final void returnIsland(Island argToRecycle) {
		Stack<Island> pool =  largePool.get().islandPool;
		pool.push(argToRecycle);
	}
	
	public static final ContactSolver getContactSolver() {
		Stack<ContactSolver> pool =  largePool.get().contactSolverPool;

		if (pool.isEmpty()) {
			pool.push(new ContactSolver());
			pool.push(new ContactSolver());
			pool.push(new ContactSolver());
			pool.push(new ContactSolver());
			pool.push(new ContactSolver());
		}

		return pool.pop();
	}

	public static final void returnContactSolver(ContactSolver argToRecycle) {
		Stack<ContactSolver> pool =  largePool.get().contactSolverPool ;
		pool.push(argToRecycle);
	}
	
	
	
	
	
	
	public static final Distance getDistance(){
		Distance distance =  largePool.get().distances ;
		return distance;
	}
	
	public static final CollideCircle getCollideCircle(){
		CollideCircle ccircle =  largePool.get().collideCircles ;
		return ccircle;
	}
	
	public static final CollidePoly getCollidePoly(){
		CollidePoly cpoly =  largePool.get().collidePolys ;
		return cpoly;
	}
}
