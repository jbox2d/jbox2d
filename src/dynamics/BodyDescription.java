package dynamics;

import collision.ShapeDescription;

import common.Settings;
import common.Vec2;

public class BodyDescription {

	ShapeDescription[] shapes;
	Vec2 position;
	float rotation;
	Vec2 linearVelocity;
	float angularVelocity;
	boolean allowSleep;
	boolean isSleeping;

	public BodyDescription() {
		shapes = new ShapeDescription[Settings.maxShapesPerBody];
		position = new Vec2(0.0f, 0.0f);
		rotation = 0.0f;
		linearVelocity = new Vec2(0.0f, 0.0f);
		angularVelocity = 0.0f;
		allowSleep = true;
		isSleeping = false;
	}

	void addShape(ShapeDescription shape) {
		for (int i = 0; i < Settings.maxShapesPerBody; ++i) {
			if (shapes[i] == null) {
				shapes[i] = shape;
				break;
			}
		}
	}
}
