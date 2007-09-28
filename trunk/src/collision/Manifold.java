package collision;

import common.Settings;
import common.Vec2;

public class Manifold {
	public ContactPoint[] points;
	public Vec2 normal;

	public Manifold() {
		points = new ContactPoint[Settings.maxManifoldPoints];
		normal = new Vec2();
	}

	public Manifold(Manifold other) {
		points = new ContactPoint[Settings.maxManifoldPoints];
		System.arraycopy(other.points, 0, points, 0, other.points.length);
		normal = other.normal.clone();
	}

}
