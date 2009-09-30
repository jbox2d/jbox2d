package org.jbox2d.pooling;

import org.jbox2d.collision.Collision;
import org.jbox2d.collision.Distance;
import org.jbox2d.collision.TOI;

public final class SingletonPool {

	private static final class Singletons{
		public final Collision collision = new Collision();
		public final Distance distance = new Distance();
		public final TOI toi = new TOI();
	}
	
	private static final class Pool extends ThreadLocal<Singletons>{
		protected Singletons initialValue(){
			return new Singletons();
		}
	}
	
	private static final Pool pool = new Pool();
	
	public static final Collision getCollision(){
		return pool.get().collision;
	}
	
	public static final Distance getDistance(){
		return pool.get().distance;
	}
	
	public static final TOI getTOI(){
		return pool.get().toi;
	}
}
