package collision;

import common.Vec2;
import java.util.ArrayList;
import java.util.List;

public class PolyDef extends ShapeDef {

    public List<Vec2> vertices;

    public PolyDef() {
        type = ShapeType.POLY_SHAPE;
        vertices = new ArrayList<Vec2>();
    }

    public int vertexCount() {
        return vertices.size();
    }
}