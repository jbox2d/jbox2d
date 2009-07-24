package org.jbox2d.common;

import java.util.HashMap;
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

public final class HashMapObjectPool {
	
	private static final HashMap<Thread, Stack<Vec2>> vec2Pool = new HashMap<Thread, Stack<Vec2>>();
	private static final HashMap<Thread, Stack<Mat22>> mat22Pool = new HashMap<Thread, Stack<Mat22>>();
	private static final HashMap<Thread, Stack<XForm>> xFormPool = new HashMap<Thread, Stack<XForm>>();
	private static final HashMap<Thread, Stack<Sweep>> sweepPool = new HashMap<Thread, Stack<Sweep>>();
	private static final HashMap<Thread, Stack<AABB>> aabbPool = new HashMap<Thread, Stack<AABB>>();
	private static final HashMap<Thread, Stack<MassData>> massPool = new HashMap<Thread, Stack<MassData>>();
	private static final HashMap<Thread, Stack<Manifold>> manifoldPool = new HashMap<Thread, Stack<Manifold>>();
	private static final HashMap<Thread, Stack<ContactPoint>> contactPointPool = new HashMap<Thread, Stack<ContactPoint>>();
	private static final HashMap<Thread, Stack<RaycastResult>> raycastResultPool = new HashMap<Thread, Stack<RaycastResult>>();
	private static final HashMap<Thread, Stack<BoundValues>> boundValuesPool = new HashMap<Thread, Stack<BoundValues>>();
	private static final HashMap<Thread, Stack<TimeStep>> timeStepPool = new HashMap<Thread, Stack<TimeStep>>();
	private static final HashMap<Thread, Stack<Integer[]>> twoIntStack = new HashMap<Thread, Stack<Integer[]>>();
	private static final HashMap<Thread, Stack<Island>> islandPool = new HashMap<Thread, Stack<Island>>();
	private static final HashMap<Thread, Stack<ContactSolver>> contactSolverPool = new HashMap<Thread, Stack<ContactSolver>>();
	
	private static final HashMap<Thread, Distance> distances = new HashMap<Thread, Distance>();
	private static final HashMap<Thread, CollideCircle> collideCircles = new HashMap<Thread, CollideCircle>();
	private static final HashMap<Thread, CollidePoly> collidePolys = new HashMap<Thread, CollidePoly>();

	
	private HashMapObjectPool() {}

	/**
	 * Initializes the pools for the thread. This would normally be in the
	 * get____() logic, but to speed things up we just have each thread call
	 * this before the pools work.
	 */
	public static final void initPools() {
		vec2Pool.put(Thread.currentThread(), new Stack<Vec2>());
		mat22Pool.put(Thread.currentThread(), new Stack<Mat22>());
		xFormPool.put(Thread.currentThread(), new Stack<XForm>());
		sweepPool.put(Thread.currentThread(), new Stack<Sweep>());
		aabbPool.put(Thread.currentThread(), new Stack<AABB>());
		massPool.put(Thread.currentThread(), new Stack<MassData>());
		manifoldPool.put(Thread.currentThread(), new Stack<Manifold>());
		contactPointPool.put(Thread.currentThread(), new Stack<ContactPoint>());
		raycastResultPool.put(Thread.currentThread(), new Stack<RaycastResult>());
		boundValuesPool.put(Thread.currentThread(), new Stack<BoundValues>());
		timeStepPool.put(Thread.currentThread(), new Stack<TimeStep>() );
		twoIntStack.put(Thread.currentThread(), new Stack<Integer[]>() );
		islandPool.put(Thread.currentThread(), new Stack<Island>());
		contactSolverPool.put(Thread.currentThread(), new Stack<ContactSolver>());
		
		distances.put(Thread.currentThread(), new Distance());
		collideCircles.put(Thread.currentThread(), new CollideCircle());
		collidePolys.put(Thread.currentThread(), new CollidePoly());
	}
	
	public static final Integer[] getTwoInts(){
		Stack<Integer[]> pool = twoIntStack.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		
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
		Stack<Integer[]> pool = twoIntStack.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		assert (argToRecycle.length == 2) : "Array has to be of 2 integers";
		pool.push(argToRecycle);
	}

	public static final Vec2 getVec2() {
		Stack<Vec2> pool = vec2Pool.get(Thread.currentThread());
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
		Stack<Vec2> pool = vec2Pool.get(Thread.currentThread());
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
		Stack<Vec2> pool = vec2Pool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}

	public static final Mat22 getMat22() {
		Stack<Mat22> pool = mat22Pool.get(Thread.currentThread());
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
		Stack<Mat22> pool = mat22Pool.get(Thread.currentThread());
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
		Stack<Mat22> pool = mat22Pool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}

	public static final XForm getXForm() {
		Stack<XForm> pool = xFormPool.get(Thread.currentThread());
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
		Stack<XForm> pool = xFormPool.get(Thread.currentThread());
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
		Stack<XForm> pool = xFormPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final Sweep getSweep() {
		Stack<Sweep> pool = sweepPool.get(Thread.currentThread());
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
		Stack<Sweep> pool = sweepPool.get(Thread.currentThread());
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
		Stack<Sweep> pool = sweepPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final AABB getAABB() {
		Stack<AABB> pool = aabbPool.get(Thread.currentThread());
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
		Stack<AABB> pool = aabbPool.get(Thread.currentThread());
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
		Stack<AABB> pool = aabbPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final MassData getMassData() {
		Stack<MassData> pool = massPool.get(Thread.currentThread());
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
		Stack<MassData> pool = massPool.get(Thread.currentThread());
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
		Stack<MassData> pool = massPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}

	public static final Manifold getManifold() {
		Stack<Manifold> pool = manifoldPool.get(Thread.currentThread());
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
		Stack<Manifold> pool = manifoldPool.get(Thread.currentThread());
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
		Stack<Manifold> pool = manifoldPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final ContactPoint getContactPoint() {
		Stack<ContactPoint> pool = contactPointPool.get(Thread.currentThread());
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
		Stack<ContactPoint> pool = contactPointPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final RaycastResult getRaycastResult() {
		Stack<RaycastResult> pool = raycastResultPool.get(Thread.currentThread());
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
		Stack<RaycastResult> pool = raycastResultPool.get(Thread.currentThread());
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
		Stack<RaycastResult> pool = raycastResultPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final BoundValues getBoundValues() {
		Stack<BoundValues> pool = boundValuesPool.get(Thread.currentThread());
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
		Stack<BoundValues> pool = boundValuesPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final TimeStep getTimeStep() {
		Stack<TimeStep> pool = timeStepPool.get(Thread.currentThread());
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
		Stack<TimeStep> pool = timeStepPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final Island getIsland() {
		Stack<Island> pool = islandPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

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
		Stack<Island> pool = islandPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	public static final ContactSolver getContactSolver() {
		Stack<ContactSolver> pool = contactSolverPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";

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
		Stack<ContactSolver> pool = contactSolverPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() once in each thread";
		pool.push(argToRecycle);
	}
	
	
	
	
	
	
	public static final Distance getDistance(){
		Distance distance = distances.get(Thread.currentThread());
		assert (distance != null) : "Distance was null, make sure you call initPools() once in each thread";
		return distance;
	}
	
	public static final CollideCircle getCollideCircle(){
		CollideCircle ccircle = collideCircles.get(Thread.currentThread());
		assert (ccircle != null) : "CollideCircle was null, make sure you call initPools() once in each thread";
		return ccircle;
	}
	
	public static final CollidePoly getCollidePoly(){
		CollidePoly cpoly = collidePolys.get(Thread.currentThread());
		assert(cpoly != null) : "CollidePoly was null, make sure you call initPools() once in each thread.";
		return cpoly;
	}
}
