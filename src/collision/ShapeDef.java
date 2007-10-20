package collision;

import java.util.ArrayList;
import java.util.List;

import common.Vec2;

public class ShapeDef {
    public ShapeType type;

    public Vec2 localPosition;
    public float localRotation;

    public float friction;
    public float restitution;
    public float density;
    
    public Object userData;
    
    public int categoryBits;
    public int maskBits;
    public int groupIndex;

    public ShapeDef() {
        // System.out.println("Unknown");
        //this(ShapeType.UNKNOWN_SHAPE);
        type = ShapeType.UNKNOWN_SHAPE;
        userData = null;
        localPosition = new Vec2();
        localRotation = 0.0f;
        friction = 0.2f;
        restitution = 0.0f;
        density = 0.0f;
        categoryBits = 0x0001;
        maskBits = 0xFFFF;
        groupIndex = 0;
    }

    public void computeMass(MassData massData) {
        if (density == 0.0f) {
            massData.mass = 0.0f;
            massData.center.set(0.0f, 0.0f);
            massData.I = 0.0f;
        }

        switch (type) {
        case CIRCLE_SHAPE:
            CircleDef circle = (CircleDef)this;
            massData.mass = (float) (density * Math.PI * circle.radius * circle.radius);
            massData.center.set(0.0f, 0.0f);
            massData.I = 0.5f * (massData.mass) * circle.radius
                    * circle.radius;
            break;

        case BOX_SHAPE:
            BoxDef box = (BoxDef)this;
            massData.mass = 4.0f * density * box.extents.x * box.extents.y;
            massData.center.set(0.0f, 0.0f);
            massData.I = massData.mass / 3.0f
                    * Vec2.dot(box.extents, box.extents);
            break;

        case POLY_SHAPE:
            PolyDef poly = (PolyDef)this;
            PolyMass(massData, poly.vertices, density);
            break;

        default:
            massData.mass = 0.0f;
            massData.center.set(0.0f, 0.0f);
            massData.I = 0.0f;
            break;
        }
    }

    private void PolyMass(MassData massData, List<Vec2> vs, float rho) {
        int count = vs.size();

        assert count >= 3;

        Vec2 center = new Vec2(0.0f, 0.0f);
        float area = 0.0f;
        float I = 0.0f;

        Vec2 pRef = new Vec2(0.0f, 0.0f);

        // #if 0 XXX ?
        // for (int i = 0; i < count; ++i) {
        // pRef.addLocal(vs.get(i));
        // }
        // pRef.mulLocal(1.0f / count);
        // #endif XXX ?

        final float inv3 = 1.0f / 3.0f;

        for (int i = 0; i < vs.size(); ++i) {
            // Triangle vertices.
            Vec2 p1 = pRef.clone();
            Vec2 p2 = vs.get(i);
            Vec2 p3 = i + 1 < count ? vs.get(i + 1) : vs.get(0);

            Vec2 e1 = p2.sub(p1);
            Vec2 e2 = p3.sub(p1);

            float D = Vec2.cross(e1, e2);

            float triangleArea = 0.5f * D;
            area += triangleArea;

            // Area weighted centroid
            // center.addLocal(p1.add(p2).add(p3).mul(triangleArea * inv3));
            center.addLocal(p1.clone().addLocal(p2).addLocal(p3).mul(
                    triangleArea * inv3));

            float px = p1.x, py = p1.y;
            float ex1 = e1.x, ey1 = e1.y;
            float ex2 = e2.x, ey2 = e2.y;

            float intx2 = inv3
                    * (0.25f * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px
                            * ex2)) + 0.5f * px * px;
            float inty2 = inv3
                    * (0.25f * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py
                            * ey2)) + 0.5f * py * py;

            I += D * (intx2 + inty2);
        }

        // Total mass
        massData.mass = rho * area;

        // Center of mass
        center.mulLocal(1.0f / area);
        massData.center = center;

        // Inertia tensor relative to the center.
        I = rho * (I - area * Vec2.dot(center, center));
        massData.I = I;
    }

}