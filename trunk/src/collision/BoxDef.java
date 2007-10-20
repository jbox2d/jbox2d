package collision;

import common.Vec2;

public class BoxDef extends ShapeDef {
    
    public Vec2 extents;
    
    public BoxDef() {
        type = ShapeType.BOX_SHAPE;
        extents = new Vec2(1.0f, 1.0f);
    }
    
}