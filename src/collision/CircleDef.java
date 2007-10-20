package collision;

public class CircleDef extends ShapeDef {

    public float radius;

    public CircleDef() {
        type = ShapeType.CIRCLE_SHAPE;
        radius = 1.0f;
    }
}