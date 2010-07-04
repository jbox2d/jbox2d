package org.jbox2d.pooling;

import org.jbox2d.collision.Distance;
import org.jbox2d.collision.shapes.CollideCircle;
import org.jbox2d.collision.shapes.CollidePoly;

public final class SingletonPool {

	private static final class Singletons{
		public final CollideCircle collideCircle = new CollideCircle();
		public final CollidePoly collidePoly = new CollidePoly();
		public final Distance distance = new Distance();
	}
	
	private static final class Pool extends ThreadLocal<Singletons>{
		protected Singletons initialValue(){
			return new Singletons();
		}
	}
	
	private static final Pool pool = new Pool();
	
	public static final CollideCircle getCollideCircle(){
		return pool.get().collideCircle;
	}
	
	public static final CollidePoly getCollidePoly(){
		return pool.get().collidePoly;
	}
	
	public static final Distance getDistance(){
		return pool.get().distance;
	}
}
