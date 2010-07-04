package dynamics.contacts;

import java.util.ArrayList;
import java.util.List;

import collision.Manifold;
import collision.Shape;
import collision.ShapeType;
import dynamics.World;

public abstract class Contact {
    public abstract void Evaluate();

    public abstract List<Manifold> GetManifolds();

    public int GetManifoldCount() {
        /*
         * List<Manifold> m = GetManifolds(); if (m == null) return 0; else
         * return GetManifolds().size();
         */
        return m_manifoldCount;
    }

    static List<ContactRegister> s_registers;

    static boolean s_initialized;

    // The parent world.
    public World m_world;

    // World pool and list pointers.
    public Contact m_prev;

    public Contact m_next;

    // Nodes for connecting bodies.
    public ContactNode m_node1;

    public ContactNode m_node2;

    public Shape m_shape1;

    public Shape m_shape2;

    // Combined friction
    public float m_friction;

    public float m_restitution;

    public boolean m_islandFlag;

    public int m_manifoldCount;

    static void InitializeRegisters() {
        s_registers = new ArrayList<ContactRegister>();

        AddType(new PolyContact(), ShapeType.POLY_SHAPE, ShapeType.POLY_SHAPE);
        AddType(new PolyContact(), ShapeType.BOX_SHAPE, ShapeType.BOX_SHAPE);
    }

    static void AddType(ContactCreator createFcn, ShapeType type1,
            ShapeType type2) {
        ContactRegister cr = new ContactRegister();
        cr.s1 = type1;
        cr.s2 = type2;
        cr.createFcn = createFcn;
        cr.primary = true;
        s_registers.add(cr);

        if (type1 != type2) {
            ContactRegister cr2 = new ContactRegister();
            cr2.s2 = type1;
            cr2.s1 = type2;
            cr2.createFcn = createFcn;
            cr2.primary = false;
            s_registers.add(cr2);
        }
    }

    public static Contact Create(Shape shape1, Shape shape2) {
        if (s_initialized == false) {
            InitializeRegisters();
            s_initialized = true;
        }

        ShapeType type1 = shape1.m_type;
        ShapeType type2 = shape2.m_type;

        // assert ShapeType.UNKNOWN_SHAPE< type1 && type1 <
        // ShapeType.SHAPE_TYPE_COUNT;
        // assert ShapeType.UNKNOWN_SHAPE < type2 && type2 <
        // ShapeType.SHAPE_TYPE_COUNT;
        ContactRegister register = getContactRegister(type1, type2);
        if (register != null) {
            if (register.primary) {
                return register.createFcn.create(shape1, shape2);
            }
            else {
                Contact c = register.createFcn.create(shape2, shape1);
                for (int i = 0; i < c.GetManifoldCount(); ++i) {
                    Manifold m = c.GetManifolds().get(i);
                    m.normal.negateLocal();
                }
                return c;
            }
        }
        else {
            return null;
        }
    }

    private static ContactRegister getContactRegister(ShapeType type1,
            ShapeType type2) {
        for (ContactRegister cr : s_registers) {
            if (cr.s1 == type1 && cr.s2 == type2) {
                return cr;
            }
        }

        return null;
    }

    public static void Destroy(Contact contact) {
        assert (s_initialized == true);

        if (contact.GetManifoldCount() > 0) {
            contact.m_shape1.m_body.wakeUp();
            contact.m_shape2.m_body.wakeUp();
        }
    }

    public Contact() {
        m_node1 = new ContactNode();
        m_node2 = new ContactNode();
    }

    public abstract Contact clone();

    public Contact(Shape s1, Shape s2) {
        this();

        // Get the shapes in a repeatable order.
        if (s1.m_body.IsStatic()) {
            assert s2.m_body.IsStatic() == false;
            m_shape1 = s1;
            m_shape2 = s2;
        }
        else if (s2.m_body.IsStatic()) {
            assert s1.m_body.IsStatic() == false;
            m_shape1 = s2;
            m_shape2 = s1;
            // } else if (s1 < s2) { //Java note: this was a pointer
            // compare...darn
        }
        else if (s1.uid < s2.uid) {// replaced with unique id
            m_shape1 = s1;
            m_shape2 = s2;
        }
        else {
            m_shape1 = s2;
            m_shape2 = s1;
        }

        // m_manifoldCount = 0;
        GetManifolds().clear();

        m_friction = (float) Math.sqrt(m_shape1.m_friction
                * m_shape2.m_friction);
        m_restitution = Math
                .max(m_shape1.m_restitution, m_shape2.m_restitution);
        m_world = s1.m_body.m_world;
        m_prev = null;
        m_next = null;
    }
}
