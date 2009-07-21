package org.jbox2d.common;

import java.util.HashMap;
import java.util.Stack;

import org.jbox2d.collision.Manifold;
import org.jbox2d.dynamics.contacts.ContactPoint;

public final class ObjectPool {
	private static final HashMap<Thread, Stack<Vec2>> vec2Pool = new HashMap<Thread, Stack<Vec2>>();
	private static final HashMap<Thread, Stack<Mat22>> mat22Pool = new HashMap<Thread, Stack<Mat22>>();
	private static final HashMap<Thread, Stack<XForm>> xFormPool = new HashMap<Thread, Stack<XForm>>();
	private static final HashMap<Thread, Stack<Sweep>> sweepPool = new HashMap<Thread, Stack<Sweep>>();
	private static final HashMap<Thread, Stack<Manifold>> manifoldPool = new HashMap<Thread, Stack<Manifold>>();
	private static final HashMap<Thread, Stack<ContactPoint>> contactPointPool = new HashMap<Thread, Stack<ContactPoint>>();
	private static final HashMap<Thread, Stack<RaycastResult>> raycastResultPool = new HashMap<Thread, Stack<RaycastResult>>();
	
	private ObjectPool() {
	};

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
		manifoldPool.put(Thread.currentThread(), new Stack<Manifold>());
		contactPointPool.put(Thread.currentThread(), new Stack<ContactPoint>());
		raycastResultPool.put(Thread.currentThread(), new Stack<RaycastResult>());
	}

	public static final Vec2 getVec2() {
		Stack<Vec2> pool = vec2Pool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";
		pool.push(argToRecycle);
	}

	public static final Mat22 getMat22() {
		Stack<Mat22> pool = mat22Pool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";
		pool.push(argToRecycle);
	}

	public static final XForm getXForm() {
		Stack<XForm> pool = xFormPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";
		pool.push(argToRecycle);
	}
	
	public static final Sweep getSweep() {
		Stack<Sweep> pool = sweepPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";
		pool.push(argToRecycle);
	}

	public static final Manifold getManifold() {
		Stack<Manifold> pool = manifoldPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";
		pool.push(argToRecycle);
	}
	
	public static final RaycastResult getRaycastResult() {
		Stack<RaycastResult> pool = raycastResultPool.get(Thread.currentThread());
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";

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
		assert (pool != null) : "Pool was null, make sure you call initPools() for each thread";
		pool.push(argToRecycle);
	}
}
