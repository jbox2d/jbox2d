package collision;

import java.util.ArrayList;
import java.util.List;

import common.Vec2;

public class ShapeDescription {
    public ShapeType type;

    public Vec2 localPosition;

    public float localRotation;

    public float friction;

    public float restitution;

    public float density;

    // TODO union
    // {
    public BoxData box;

    public CircleData circle;

    public PolyData poly;

    // };

    public ShapeDescription() {
        // System.out.println("Unknown");
        this(ShapeType.UNKNOWN_SHAPE);
    }

    public ShapeDescription(ShapeType s) {
        this.type = s;
        switch (s) {
        case CIRCLE_SHAPE:
            // System.out.println("circ");
            circle = new CircleData();
            break;

        case BOX_SHAPE:
            // System.out.println("box");
            box = new BoxData();
            break;

        case POLY_SHAPE:
            // System.out.println("poly");
            poly = new PolyData();
            break;

        default:
            circle = new CircleData();
            box = new BoxData();
            poly = new PolyData();
        }

        localPosition = new Vec2(0.0f, 0.0f);
        localRotation = 0.0f;
        friction = 0.2f;
        restitution = 0.0f;
        density = 0.0f;
    }

    public void computeMass(MassData massData) {
        if (density == 0.0f) {
            massData.mass = 0.0f;
            massData.center.set(0.0f, 0.0f);
            massData.I = 0.0f;
        }

        switch (type) {
        case CIRCLE_SHAPE:
            massData.mass = (float) (density * Math.PI * circle.m_radius * circle.m_radius);
            massData.center.set(0.0f, 0.0f);
            massData.I = 0.5f * (massData.mass) * circle.m_radius
                    * circle.m_radius;
            break;

        case BOX_SHAPE:
            massData.mass = 4.0f * density * box.m_extents.x * box.m_extents.y;
            massData.center.set(0.0f, 0.0f);
            massData.I = massData.mass / 3.0f
                    * Vec2.dot(box.m_extents, box.m_extents);
            break;

        case POLY_SHAPE:
            PolyMass(massData, poly.m_vertices, density);
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

        Vec2 a = new Vec2(0.0f, 0.0f);

        // #if 0 XXX ?
        // for (int i = 0; i < count; ++i) {
        // a.addLocal(vs.get(i));
        // }
        // a.mulLocal(1.0f / count);
        // #endif XXX ?

        final float inv3 = 1.0f / 3.0f;

        for (int i = 0; i < vs.size(); ++i) {
            // Triangle vertices.
            Vec2 p1 = a;
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

    public class BoxData {
        public Vec2 m_extents;

        public BoxData() {
            m_extents = new Vec2();
        }

    };

    public class CircleData {
        public float m_radius;

        public CircleData() {

        }
    };

    // Convex polygon, vertices must be in CCW order.
    public class PolyData {
        public List<Vec2> m_vertices;

        // b2Vec2 m_vertices[b2_maxPolyVertices];
        // int32 m_vertexCount;

        public PolyData() {
            m_vertices = new ArrayList<Vec2>();
        }

    };
}