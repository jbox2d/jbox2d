package testbed;

import java.awt.Color;
import java.awt.Frame;
import java.awt.Graphics2D;
import java.awt.Toolkit;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferStrategy;
import java.util.Random;

import collision.AABB;
import collision.BroadPhase;
import collision.Manifold;
import collision.Pair;
import collision.PolyShape;
import collision.Proxy;
import collision.Shape;
import collision.ShapeDescription;
import collision.ShapeType;

import common.Settings;
import common.Vec2;

import dynamics.Body;
import dynamics.BodyDescription;
import dynamics.World;
import dynamics.contacts.Contact;
import dynamics.joints.Joint;
import dynamics.joints.JointType;
import dynamics.joints.MouseDescription;
import dynamics.joints.MouseJoint;

public abstract class Test {
    /** The frame displaying the demo */
    protected Frame frame;

    /** The title of the current demo */
    protected String title;

    /** True if the simulation is running */
    private boolean running = true;

    /** The rendering strategy */
    private BufferStrategy strategy;

    /** True if we should reset the demo on the next loop */
    protected boolean needsReset;

    /** True if we should render normals */
    private boolean normals = true;

    /** True if we should render contact points */
    private boolean contacts = true;

    protected int m_textLine;

    protected World m_world;

    protected Body m_bomb;

    protected MouseJoint m_mouseJoint;

    public Test() {
        this("JBox2D test");
    }

    /**
     * Create a new demo
     * 
     * @param title
     *            The title of the demo
     */
    public Test(String title) {
        this.title = title;

        m_world = new World(new AABB(new Vec2(-100, -100), new Vec2(100, 100)),
                new Vec2(0.0f, -10.0f), true);

        m_bomb = null;
        m_textLine = 30;
        m_mouseJoint = null;

    }

    void MouseDown(Vec2 p) {
        assert m_mouseJoint == null;

        // Make a small box.

        Vec2 d = new Vec2(0.001f, 0.001f);
        AABB aabb = new AABB(p.sub(d), p.add(d));

        // Query the world for overlapping shapes.
        int k_maxCount = 10;
        Shape shapes[] = m_world.Query(aabb, k_maxCount);
        Body body = null;
        for (int j = 0; j < shapes.length; j++) {
            Shape shape = shapes[j];
            if (!shape.m_body.IsStatic()) {
                boolean inside = shape.TestPoint(p);
                if (inside) {
                    body = shape.m_body;
                    break;
                }
            }
        }

        if (body != null) {
            MouseDescription md = new MouseDescription();
            md.body1 = m_world.m_groundBody;
            md.body2 = body;
            md.target = p;
            md.motorForce = 400.0f * body.m_mass;
            m_mouseJoint = (MouseJoint) m_world.CreateJoint(md);
            body.wakeUp();
        }
    }

    void MouseUp() {
        if (m_mouseJoint != null) {
            m_world.DestroyJoint(m_mouseJoint);
            m_mouseJoint = null;
        }
    }

    void MouseMove(Vec2 p) {
        if (m_mouseJoint != null) {
            m_mouseJoint.SetTarget(p);
        }
    }

    void LaunchBomb() {
        if (m_bomb == null) {
            ShapeDescription sd = new ShapeDescription();
            float a = 0.5f;
            sd.type = ShapeType.BOX_SHAPE;
            sd.box.m_extents = new Vec2(a, a);
            sd.density = 20.0f;

            BodyDescription bd = new BodyDescription();
            bd.addShape(sd);
            m_bomb = m_world.CreateBody(bd);
        }

        Random r = new Random();

        Vec2 position = new Vec2(r.nextFloat() * 30 - 15, 30.0f);
        float rotation = r.nextFloat() * 3 - 1.5f;
        m_bomb.SetRootPosition(position, rotation);
        m_bomb.m_linearVelocity = position.mul(-1.0f);
        m_bomb.m_angularVelocity = r.nextFloat() * 40 - 20;
        m_bomb.wakeUp();
    }

    void Step(TestSettings settings, Graphics2D g) {
        float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

        World.s_enableWarmStarting = settings.enableWarmStarting;
        World.s_enablePositionCorrection = settings.enablePositionCorrection;

        m_world.Step(timeStep, settings.iterationCount);

        // m_world.m_broadPhase.Validate();

        for (Body b = m_world.m_bodyList; b != null; b = b.m_next) {
            for (Shape s = b.m_shapeList; s != null; s = s.m_next) {
                if (b.m_invMass == 0.0f) {
                    DrawShape(s, new Color(0.5f, 0.9f, 0.5f), g);
                }
                else if (b.m_isSleeping) {
                    DrawShape(s, new Color(0.5f, 0.5f, 0.9f), g);
                }
                else if (b == m_bomb) {
                    DrawShape(s, new Color(0.9f, 0.9f, 0.4f), g);
                }
                else {
                    DrawShape(s, new Color(0.9f, 0.9f, 0.9f), g);

                }
            }
        }

        for (Joint j = m_world.m_jointList; j != null; j = j.m_next) {
            if (j != m_mouseJoint) {
                DrawJoint(j, g);
            }
        }

        if (settings.drawContacts) {
            for (Contact c = m_world.m_contactList; c != null; c = c.m_next) {
                drawContact(c, g);
            }
        }

        if (settings.drawImpulses) {
            for (Contact c = m_world.m_contactList; c != null; c = c.m_next) {
                drawImpulse(c, g);
            }
        }

        if (settings.drawPairs) {
            drawPairs(g);
        }

        if (settings.drawAABBs) {
            BroadPhase bp = m_world.m_broadPhase;
            Vec2 invQ = new Vec2(1.0f / bp.m_quantizationFactor.x,
                    1.0f / bp.m_quantizationFactor.y);
            for (int i = 0; i < Settings.maxProxies; ++i) {
                Proxy p = bp.m_proxyPool[i];
                if (p.IsValid() == false) {
                    continue;
                }

                AABB b = new AABB();
                b.minVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                        * bp.m_bounds[0][p.lowerBounds[0]].value;
                b.minVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                        * bp.m_bounds[1][p.lowerBounds[1]].value;
                b.maxVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                        * bp.m_bounds[0][p.upperBounds[0]].value;
                b.maxVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                        * bp.m_bounds[1][p.upperBounds[1]].value;

                DrawAABB(b, Color.pink, g);
            }
        }

        if (settings.drawStats) {
            g.drawString("proxies(max) = " + m_world.m_broadPhase.m_proxyCount
                    + "(" + Settings.maxProxies + "), pairs(max) = "
                    + m_world.m_broadPhase.m_pairManager.m_pairCount + "("
                    + Settings.maxPairs + ")", 5, m_textLine);

            m_textLine += 15;

            g.drawString(
                    "bodies/contacts/joints = " + m_world.m_bodyCount + "/"
                            + m_world.m_contactCount + "/"
                            + m_world.m_jointCount, 5, m_textLine);
        }

        if (m_mouseJoint != null) {
            drawMouseJoint(g);
        }
    }

    void DrawShape(Shape shape, Color c, Graphics2D g) {
        g.setColor(c);
        if (shape.m_type == ShapeType.POLY_SHAPE
                || shape.m_type == ShapeType.BOX_SHAPE) {
            PolyShape poly = (PolyShape) shape;

            int x[] = new int[poly.m_vertexCount];
            int y[] = new int[poly.m_vertexCount];

            for (int i = 0; i < poly.m_vertexCount; ++i) {
                Vec2 v = poly.m_position.add(poly.m_R.mul(poly.m_vertices[i]));

                x[i] = (int) v.x;
                y[i] = (int) v.y;
            }
            g.drawPolygon(x, y, poly.m_vertexCount);
        }
    }

    void DrawJoint(Joint joint, Graphics2D g) {
        Body b1 = joint.m_body1;
        Body b2 = joint.m_body2;
        Vec2 x1 = b1.m_position;
        Vec2 x2 = b2.m_position;
        Vec2 p1 = joint.GetAnchor1();
        Vec2 p2 = joint.GetAnchor2();

        g.setColor(new Color(0.5f, 0.8f, 0.8f));

        if (joint.m_type == JointType.distanceJoint) {
            g.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }
        else {
            g.drawLine((int) x1.x, (int) x1.y, (int) p1.x, (int) p1.y);
            g.drawLine((int) x2.x, (int) x2.y, (int) p2.x, (int) p2.y);
        }
    }

    void DrawAABB(AABB aabb, Color c, Graphics2D g) {
        g.setColor(c);

        g.drawLine((int) aabb.minVertex.x, (int) aabb.minVertex.y,
                (int) aabb.maxVertex.x, (int) aabb.minVertex.y);
        g.drawLine((int) aabb.maxVertex.x, (int) aabb.minVertex.y,
                (int) aabb.maxVertex.x, (int) aabb.maxVertex.y);
        g.drawLine((int) aabb.maxVertex.x, (int) aabb.maxVertex.y,
                (int) aabb.minVertex.x, (int) aabb.maxVertex.y);
        g.drawLine((int) aabb.minVertex.x, (int) aabb.maxVertex.y,
                (int) aabb.minVertex.x, (int) aabb.minVertex.y);
    }

    void drawContact(Contact c, Graphics2D g) {
        g.setColor(new Color(1.0f, 0.0f, 0.0f));
        for (Manifold m : c.GetManifolds()) {
            for (int j = 0; j < m.pointCount; ++j) {
                Vec2 v = m.points[j].position;
                g.fillOval((int) v.x, (int) v.y, 4, 4);
            }
        }
    }

    void drawImpulse(Contact c, Graphics2D g) {
        g.setColor(new Color(0.9f, 0.9f, 0.3f));
        for (Manifold m : c.GetManifolds()) {
            for (int j = 0; j < m.pointCount; ++j) {
                Vec2 v1 = m.points[j].position;
                Vec2 v2 = v1.add(m.normal.mul(m.points[j].normalImpulse));

                g.drawLine((int) v1.x, (int) v1.y, (int) v2.x, (int) v2.y);
            }
        }
    }

    void drawPairs(Graphics2D g) {
        g.setColor(new Color(0.9f, 0.9f, 0.3f));
        BroadPhase bp = m_world.m_broadPhase;
        Vec2 invQ = new Vec2(1.0f / bp.m_quantizationFactor.x,
                1.0f / bp.m_quantizationFactor.y);
        for (int i = 0; i < bp.m_pairManager.m_pairCount; ++i) {
            Pair pair = bp.m_pairManager.m_pairs[i];
            int id1 = pair.proxyId1;
            int id2 = pair.proxyId2;
            Proxy p1 = bp.m_proxyPool[id1];
            Proxy p2 = bp.m_proxyPool[id2];

            AABB b1 = new AABB();
            AABB b2 = new AABB();
            b1.minVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                    * bp.m_bounds[0][p1.lowerBounds[0]].value;
            b1.minVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                    * bp.m_bounds[1][p1.lowerBounds[1]].value;
            b1.maxVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                    * bp.m_bounds[0][p1.upperBounds[0]].value;
            b1.maxVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                    * bp.m_bounds[1][p1.upperBounds[1]].value;
            b2.minVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                    * bp.m_bounds[0][p2.lowerBounds[0]].value;
            b2.minVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                    * bp.m_bounds[1][p2.lowerBounds[1]].value;
            b2.maxVertex.x = bp.m_worldAABB.minVertex.x + invQ.x
                    * bp.m_bounds[0][p2.upperBounds[0]].value;
            b2.maxVertex.y = bp.m_worldAABB.minVertex.y + invQ.y
                    * bp.m_bounds[1][p2.upperBounds[1]].value;

            Vec2 x1 = b1.minVertex.add(b1.maxVertex).mul(0.5f);
            Vec2 x2 = b2.minVertex.add(b2.maxVertex).mul(0.5f);

            g.drawLine((int) x1.x, (int) x1.y, (int) x2.x, (int) x2.y);
        }
    }

    void drawMouseJoint(Graphics2D g) {
        Body body = m_mouseJoint.m_body2;
        Vec2 p1 = body.m_position.add(body.m_R.mul(m_mouseJoint.m_localAnchor));
        Vec2 p2 = m_mouseJoint.m_target;

        g.setColor(new Color(0.0f, 1.0f, 0.0f));
        g.fillOval((int) p1.x, (int) p1.y, 4, 4);
        g.fillOval((int) p2.x, (int) p2.y, 4, 4);

        g.setColor(new Color(0.8f, 0.8f, 0.8f));
        g.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
    }

    /**
     * Retrieve the title of the demo
     * 
     * @return The title of the demo
     */
    public String getTitle() {
        return title;
    }

    /**
     * Notification that a key was pressed
     * 
     * @param c
     *            The character of key hit
     */
    protected void keyHit(char c) {
        if (c == 'r') {
            needsReset = true;
        }
        if (c == 'c') {
            normals = !normals;
            contacts = !contacts;
        }
    }

    /**
     * Initialise the GUI
     */
    private void initGUI() {
        frame = new Frame(title);
        frame.setResizable(false);
        frame.setIgnoreRepaint(true);
        frame.setSize(500, 500);

        int x = (int) (Toolkit.getDefaultToolkit().getScreenSize().getWidth() - 500) / 2;
        int y = (int) (Toolkit.getDefaultToolkit().getScreenSize().getHeight() - 500) / 2;

        frame.setLocation(x, y);

        frame.addWindowListener(new WindowAdapter() {
            public void windowClosing(WindowEvent e) {
                running = false;
                System.exit(0);
            }
        });
        frame.addKeyListener(new KeyAdapter() {
            public void keyTyped(KeyEvent e) {
                keyHit(e.getKeyChar());
            }

            public void keyPressed(KeyEvent e) {
                if (e.getKeyCode() == 27) {
                    System.exit(0);
                }
            }

        });

        frame.setVisible(true);
        frame.createBufferStrategy(2);

        strategy = frame.getBufferStrategy();
    }

    /**
     * Start the simulation running
     */
    public void start() {
        initGUI();
        initDemo();

        float target = 1000 / 60.0f;
        float frameAverage = target;
        long lastFrame = System.currentTimeMillis();
        float yield = 10000f;
        float damping = 0.1f;

        long renderTime = 0;
        long logicTime = 0;

        TestSettings settings = new TestSettings();

        while (running) {
            // adaptive timing loop from Master Onyx
            long timeNow = System.currentTimeMillis();
            frameAverage = (frameAverage * 10 + (timeNow - lastFrame)) / 11;
            lastFrame = timeNow;

            yield += yield * ((target / frameAverage) - 1) * damping + 0.05f;

            for (int i = 0; i < yield; i++) {
                Thread.yield();
            }

            // render
            long beforeRender = System.currentTimeMillis();
            Graphics2D g = (Graphics2D) strategy.getDrawGraphics();
            g.setColor(Color.white);
            g.fillRect(0, 0, 500, 500);

            g.translate(300, 100);
            g.scale(2.0, 2.0);
            g.rotate(Math.PI);

            // update data model
            long beforeLogic = System.currentTimeMillis();
            // for (int i = 0; i < 4; i++) {
            // m_world.Step(16, 10);
            // }
            Step(settings, g);
            logicTime = System.currentTimeMillis() - beforeLogic;

            g.setColor(Color.black);
            // g.drawString("FAv: " + frameAverage, 10, 50);
            // g.drawString("FPS: " + (int) (1000 / frameAverage), 10, 70);
            // g.drawString("Yield: " + yield, 10, 90);
            // g.drawString("Bodies: " + m_world.getBodies().size(), 10,
            // 130);
            // g.drawString("R: " + renderTime, 10, 150);
            // g.drawString("L: " + logicTime, 10, 170);
            // g.drawString("Energy: " + m_world.getTotalEnergy(), 10, 190);

            renderGUI(g);
            g.dispose();
            strategy.show();
            renderTime = System.currentTimeMillis() - beforeRender;

            if (needsReset) {
                // XXX m_world.clear();
                initDemo();
                needsReset = false;
                frameAverage = target;
                yield = 10000f;
            }

            update();
        }
    }

    /**
     * Update the demo - just in case we want to add anything over the top
     */
    protected void update() {
    }

    /**
     * Demo customisable GUI render
     * 
     * @param g
     *            The graphics context to use for rendering here
     */
    protected void renderGUI(Graphics2D g) {
        g.setColor(Color.black);
        g.drawString("R - Restart Demo", 15, 430);
    }

    /**
     * Initialise the demo - clear the world
     */
    public final void initDemo() {
        // XXX m_world.clear();

        System.out.println("Initialising:" + getTitle());
        init(m_world);
    }

    /**
     * Should be implemented by the demo, add the bodies/joints to the world.
     * 
     * @param world
     *            The world in which the simulation is going to run
     */
    protected abstract void init(World world);
}
