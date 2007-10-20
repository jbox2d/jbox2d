package dynamics;

import collision.ShapeDef;

import common.Settings;
import common.Vec2;

public class BodyDef {
    
    public Object userData;

    public ShapeDef[] shapes;

    public Vec2 position;

    public float rotation;

    public Vec2 linearVelocity;

    public float angularVelocity;

    public boolean allowSleep;

    public boolean isSleeping;
    
    public boolean preventRotation;

    public BodyDef() {
        userData = null;
        shapes = new ShapeDef[Settings.maxShapesPerBody];
        position = new Vec2(0.0f, 0.0f);
        rotation = 0.0f;
        linearVelocity = new Vec2(0.0f, 0.0f);
        angularVelocity = 0.0f;
        allowSleep = true;
        isSleeping = false;
        preventRotation = false;
    }

    public void addShape(ShapeDef shape) {
        for (int i = 0; i < Settings.maxShapesPerBody; ++i) {
            if (shapes[i] == null) {
                shapes[i] = shape;
                // System.out.println(shape.type);
                break;
            }
        }
    }
}
