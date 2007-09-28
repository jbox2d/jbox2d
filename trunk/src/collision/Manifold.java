package collision;

import java.util.ArrayList;
import java.util.List;

import common.Settings;
import common.Vec2;

public class Manifold {
	public List<ContactPoint> points;
	public Vec2 normal;

	public Manifold() {
		points = new ArrayList<ContactPoint>(Settings.maxManifoldPoints);
		normal = new Vec2();
	}
	
	public Manifold(Manifold other) {
		points = new ArrayList<ContactPoint>(other.points);
		normal = other.normal.clone();
	}

}
