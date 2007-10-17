package dynamics;

import collision.ShapeDescription;

import common.Settings;
import common.Vec2;

public class BodyDescription {

    public ShapeDescription[] shapes;

    public Vec2 position;

    public float rotation;

    public Vec2 linearVelocity;

    public float angularVelocity;

    public boolean allowSleep;

    public boolean isSleeping;

    public BodyDescription() {
        shapes = new ShapeDescription[Settings.maxShapesPerBody];
        position = new Vec2(0.0f, 0.0f);
        rotation = 0.0f;
        linearVelocity = new Vec2(0.0f, 0.0f);
        angularVelocity = 0.0f;
        allowSleep = true;
        isSleeping = false;
    }

    public void addShape(ShapeDescription shape) {
        for (int i = 0; i < Settings.maxShapesPerBody; ++i) {
            if (shapes[i] == null) {
                shapes[i] = shape;
                // System.out.println(shape.type);
                break;
            }
        }
    }
}
