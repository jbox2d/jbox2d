/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package testbed;

import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.util.Random;
import java.util.ArrayList;

import processing.core.PApplet;
import processing.core.PImage;

import collision.AABB;
import collision.BoxDef;
import collision.BroadPhase;
import collision.CircleShape;
import collision.Manifold;
import collision.Pair;
import collision.PolyShape;
import collision.Proxy;
import collision.Shape;
import collision.ShapeType;

import common.Settings;
import common.Vec2;

import dynamics.Body;
import dynamics.BodyDef;
import dynamics.World;
import dynamics.contacts.Contact;
import dynamics.joints.Joint;
import dynamics.joints.JointType;
import dynamics.joints.MouseJointDef;
import dynamics.joints.MouseJoint;
import dynamics.joints.PulleyJoint;

public abstract class PTest extends PApplet {
    
    public Body followedBody;

    public boolean[] keyDown;

    public boolean[] newKeyDown;

    // World 0,0 maps to transX, transY on screen
    public float transX;

    public float transY;

    public float scaleFactor = 10.0f;

    public Vec2 mouseWorld; // world coordinates of mouse

    public boolean pmousePressed; // was mouse pressed last frame?

    static public void main(String args[]) {
        PApplet.main(new String[] { "testbed.PTest" });
    }

    protected TestSettings settings;

    /** The title of the current demo */
    protected String title;

    /** True if we should reset the demo on the next loop */
    protected boolean needsReset;

    /** True if we should render normals */
    private boolean normals = false;

    /** True if we should render contact points */
    private boolean contacts = false;

    protected int m_textLine;

    protected World m_world;

    protected Body m_bomb;

    protected MouseJoint m_mouseJoint;
    
    public ArrayList<BoundImage> boundImages = new ArrayList<BoundImage>();

    public PTest() {
        this("JBox2D test");
    }

    /**
     * Create a new demo
     * 
     * @param title
     *            The title of the demo
     */
    public PTest(String title) {
        this.title = title;

        setupWorld();

        m_textLine = 30;
        //System.out.println("Constructing PTest");
    }

    void MouseDown(Vec2 p) {
        assert m_mouseJoint == null;

        // Make a small box.

        Vec2 d = new Vec2(0.001f, 0.001f);
        AABB aabb = new AABB(p.sub(d), p.add(d));

        // Query the world for overlapping shapes.
        int k_maxCount = 10;
        Shape shapes[] = m_world.query(aabb, k_maxCount);
        Body body = null;
        for (int j = 0; j < shapes.length; j++) {
            Shape shape = shapes[j];
            if (shape != null && !shape.m_body.isStatic()) {
                boolean inside = shape.testPoint(p);
                if (inside) {
                    body = shape.m_body;
                    break;
                }
            }
        }

        if (body != null) {
            MouseJointDef md = new MouseJointDef();
            md.body1 = m_world.m_groundBody;
            md.body2 = body;
            md.target = p;
            md.maxForce = 1000.0f * body.m_mass;
            m_mouseJoint = (MouseJoint) m_world.createJoint(md);
            body.wakeUp();
        }
    }

    void MouseUp() {
        if (m_mouseJoint != null) {
            m_world.destroyJoint(m_mouseJoint);
            m_mouseJoint = null;
        }
    }

    void MouseMove(Vec2 p) {
        if (m_mouseJoint != null) {
            m_mouseJoint.setTarget(p);
        }
    }

    void LaunchBomb() {
        if (m_bomb != null && m_bomb.isFrozen()) {
            m_world.destroyBody(m_bomb);
            m_world.cleanBodyList();
            m_bomb = null;
        }
        if (m_bomb == null) {
            BoxDef sd = new BoxDef();
            float a = 0.5f;
            // sd.type = ShapeType.BOX_SHAPE;

            sd.extents = new Vec2(a, a);
            sd.density = 20.0f;

            BodyDef bd = new BodyDef();
            bd.addShape(sd);
            m_bomb = m_world.createBody(bd);
        }

        Random r = new Random();

        Vec2 position = new Vec2(r.nextFloat() * 30 - 15, 30.0f);
        float rotation = r.nextFloat() * 3 - 1.5f;
        m_bomb.setOriginPosition(position, rotation);
        m_bomb.m_linearVelocity = position.mul(-1.0f);
        m_bomb.m_angularVelocity = r.nextFloat() * 40 - 20;
        m_bomb.wakeUp();
    }

    void Step(TestSettings settings) {
        float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

        World.ENABLE_WARM_STARTING = settings.enableWarmStarting;
        World.ENABLE_POSITION_CORRECTION = settings.enablePositionCorrection;
        //println("pre");
        m_world.step(timeStep, settings.iterationCount);
        //println("post");
        // m_world.m_broadPhase.Validate();

        for (Body b = m_world.m_bodyList; b != null; b = b.m_next) {
            for (Shape s = b.m_shapeList; s != null; s = s.m_next) {
                if (b.m_invMass == 0.0f) {
                    DrawShape(s, color(100, 100, 100));
                }
                else if (b.isSleeping()) {
                    DrawShape(s, color(30, 30, 90));
                }
                else if (b == m_bomb) {
                    DrawShape(s, color(255, 0, 0));
                }
                else {
                    DrawShape(s, color(30, 30, 30));

                }
            }
            if (settings.drawCM) {
                noFill();
                stroke(0,0,250);
                ellipse(b.m_position.x, b.m_position.y, 2f/scaleFactor, 2f/scaleFactor);
            }
        }
        
        for (BoundImage b:boundImages) {
            b.draw();
        }

        for (Joint j = m_world.m_jointList; j != null; j = j.m_next) {
            if (j != m_mouseJoint) {
                DrawJoint(j);
            }
        }

        if (settings.drawContacts) {
            for (Contact c = m_world.m_contactList; c != null; c = c.m_next) {
                drawContact(c);
            }
        }

        if (settings.drawImpulses) {
            for (Contact c = m_world.m_contactList; c != null; c = c.m_next) {
                drawImpulse(c);
            }
        }

        if (settings.drawPairs) {
            drawPairs();
        }
        DrawAABB(m_world.m_broadPhase.m_worldAABB, color(0, 255, 0));
        if (settings.drawAABBs) {
            BroadPhase bp = m_world.m_broadPhase;
            Vec2 invQ = new Vec2(1.0f / bp.m_quantizationFactor.x,
                    1.0f / bp.m_quantizationFactor.y);
            for (int i = 0; i < Settings.maxProxies; ++i) {
                Proxy p = bp.m_proxyPool[i];
                if (p.isValid() == false) {
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

                DrawAABB(b, color(255, 150, 150));
            }
        }

        /*
         * if (settings.drawStats) { g.drawString("proxies(max) = " +
         * m_world.m_broadPhase.m_proxyCount + "(" + Settings.maxProxies + "),
         * pairs(max) = " + m_world.m_broadPhase.m_pairManager.m_pairCount + "(" +
         * Settings.maxPairs + ")", 5, m_textLine);
         * 
         * m_textLine += 15;
         * 
         * g.drawString( "bodies/contacts/joints = " + m_world.m_bodyCount + "/" +
         * m_world.m_contactCount + "/" + m_world.m_jointCount, 5, m_textLine); }
         */

        if (m_mouseJoint != null) {
            drawMouseJoint();
        }
    }
    
//    public void DrawImage(PImage p, Vec2[] localPoints, Vec2[] texCoords, Body b) {
//        if (localPoints.length != 4 || texCoords.length != 4) assert false;
//        beginShape(POLYGON);
//        texture(p);
//        noStroke();
//        fill(255);
//        for (int i=0; i < 4; ++i) {
//            Vec2 worldPt = b.getWorldPoint(localPoints[i]);
//            println(worldPt.x + " " +worldPt.y);
//            vertex(worldPt.x, worldPt.y, texCoords[i].x, texCoords[i].y);
//        }
//        endShape();
//    }

    /**
     * Draws an image on a body.
     * 
     * First image is centered on body center, then
     * localScale is applied, then localOffset, and
     * lastly localRotation (all rel. to body center).
     * 
     * Thus localOffset should be specified in body
     * units to the scaled image.  For instance, if
     * you want a MxN image to have its corner
     * at body center and be scaled by S, use a localOffset
     * of (M*S/2, N*S/2) and a localScale of S.
     * 
     */
    public void bindImage(PImage p, Vec2 localOffset, float localRotation, float localScale, Body b) {
        boundImages.add(new BoundImage(p, localOffset, localRotation, localScale, b));
    }
    
    
    class BoundImage{
        private PImage image;
        private float halfImageWidth;
        private float halfImageHeight;
        public Body body;
        public Vec2 localOffset;
        public float localRotation;
        public float localScale;
        
        public BoundImage(PImage _image, Vec2 _localOffset, float _localRotation, float _localScale, Body _body) {
            image = _image;
            localOffset = _localOffset.clone();
            localRotation = _localRotation;
            localScale = _localScale;
            body = _body;
            halfImageWidth = image.width / 2f;
            halfImageHeight = image.height / 2f;
        }
        
        public void draw() {
            pushMatrix();
            translate(body.m_position.x, body.m_position.y);
            rotate(body.m_rotation+localRotation);
            translate(localOffset.x, localOffset.y);
            scale(localScale);
            image(image, -halfImageWidth, -halfImageHeight);
            popMatrix();
        }
    }
    
    public void DrawShape(Shape shape, int c) {
        stroke(c);
        noFill();
        if (shape.m_type == ShapeType.POLY_SHAPE
                || shape.m_type == ShapeType.BOX_SHAPE) {
            PolyShape poly = (PolyShape) shape;

            beginShape(POLYGON);
            for (int i = 0; i < poly.m_vertexCount; ++i) {
                Vec2 v = poly.m_R.mul(poly.m_vertices[i]).addLocal(
                        poly.m_position);
                vertex(v.x, v.y);
            }
            Vec2 v = poly.m_R.mul(poly.m_vertices[0]).addLocal(poly.m_position);
            vertex(v.x, v.y);
            endShape();
        }
        else if (shape.m_type == ShapeType.CIRCLE_SHAPE) {
            CircleShape circle = (CircleShape) shape;
            ellipse(circle.m_position.x, circle.m_position.y,
                    circle.m_radius * 2, circle.m_radius * 2);
            float xR = circle.m_radius
                    * (float) Math.cos(shape.m_body.m_rotation);
            float yR = circle.m_radius
                    * (float) Math.sin(shape.m_body.m_rotation);
            line(circle.m_position.x, circle.m_position.y, circle.m_position.x
                    + xR, circle.m_position.y + yR);
        }
    }

    public void DrawJoint(Joint joint) {
        Body b1 = joint.m_body1;
        Body b2 = joint.m_body2;
        Vec2 x1 = b1.m_position;
        Vec2 x2 = b2.m_position;
        Vec2 p1 = joint.getAnchor1();
        Vec2 p2 = joint.getAnchor2();

        stroke(0.5f, 0.8f, 0.8f);
        noFill();

        if (joint.m_type == JointType.DISTANCE_JOINT) {
            line(p1.x, p1.y, p2.x, p2.y);
        }
        else if (joint.m_type == JointType.PULLEY_JOINT) {
            PulleyJoint pj = (PulleyJoint) joint;
            line(x1.x, x1.y, pj.m_groundAnchor1.x, pj.m_groundAnchor1.y);
            line(x2.x, x2.y, pj.m_groundAnchor2.x, pj.m_groundAnchor2.y);
        }
        else if (joint.m_type == JointType.PRISMATIC_JOINT) {
            // TODO: draw Prismatic Joint
        }
        else if (joint.m_type == JointType.GEAR_JOINT) {
            // TODO?: draw Gear Joint? (not much to draw...)
        }
        else {
            line(x1.x, x1.y, p1.x, p1.y);
            line(x2.x, x2.y, p2.x, p2.y);
        }
    }

    void DrawAABB(AABB aabb, int c) {
        stroke(c);
        noFill();
        rect(aabb.minVertex.x, aabb.minVertex.y,
                (aabb.maxVertex.x - aabb.minVertex.x),
                (aabb.maxVertex.y - aabb.minVertex.y));
    }

    void drawContact(Contact c) {
        // g.setColor(new Color(1.0f, 0.0f, 0.0f));
        fill(255, 0, 0);
        noStroke();
        for (Manifold m : c.GetManifolds()) {
            for (int j = 0; j < m.pointCount; ++j) {
                Vec2 v = m.points[j].position;
                // g.fillOval((int) v.x, (int) v.y, 4, 4);
                ellipse(v.x, v.y, 4f / scaleFactor, 4f / scaleFactor);
            }
        }
    }

    void drawImpulse(Contact c) {
        // g.setColor(new Color(0.9f, 0.9f, 0.3f));
        stroke(230, 230, 80);
        for (Manifold m : c.GetManifolds()) {
            for (int j = 0; j < m.pointCount; ++j) {
                Vec2 v1 = m.points[j].position;
                Vec2 v2 = v1.add(m.normal.mul(m.points[j].normalImpulse));

                // g.drawLine((int) v1.x, (int) v1.y, (int) v2.x, (int) v2.y);
                line(v1.x, v1.y, v2.x, v2.y);
            }
        }
    }

    void drawPairs() {
        // g.setColor(new Color(0.9f, 0.9f, 0.3f));
        stroke(230, 230, 80);
        BroadPhase bp = m_world.m_broadPhase;
        Vec2 invQ = new Vec2(1.0f / bp.m_quantizationFactor.x,
                1.0f / bp.m_quantizationFactor.y);
        for (int i = 0; i < bp.m_pairManager.m_pairCount; ++i) {
            Pair pair = bp.m_pairManager.m_pairs[i];
            int id1 = pair.proxyId1;
            int id2 = pair.proxyId2;
            if (id1 > bp.m_proxyPool.length || id2 > bp.m_proxyPool.length) continue;
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

            // g.drawLine((int) x1.x, (int) x1.y, (int) x2.x, (int) x2.y);
            line(x1.x, x1.y, x2.x, x2.y);
        }
    }

    void drawMouseJoint() {
        Body body = m_mouseJoint.m_body2;
        Vec2 p1 = body.m_R.mul(m_mouseJoint.m_localAnchor).addLocal(
                body.m_position);
        Vec2 p2 = m_mouseJoint.m_target;

        fill(0, 255, 0);
        noStroke();

        float diam = 5.0f / scaleFactor;
        ellipse(p1.x, p1.y, diam, diam);
        ellipse(p2.x, p2.y, diam, diam);

        stroke(200, 200, 200);
        line(p1.x, p1.y, p2.x, p2.y);
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
     * Notification that a key was typed.
     * 
     */
    public void keyTyped() {
        if (key == 'r') {
            needsReset = true;
        }
        if (key == 'c') {
            normals = !normals;
            contacts = !contacts;
            // settings.drawNormals = normals;
            settings.drawContacts = contacts;
        }
        if (key == 'b') {
            settings.drawAABBs = !settings.drawAABBs;
        }
        if (key == ' ') {
            LaunchBomb();
        }
    }

    public void keyPressed() {
        if (key >= 0 && key < 255) {
            if (!keyDown[key])
                newKeyDown[key] = true;
            keyDown[key] = true;
        }
    }

    public void keyReleased() {
        if (key >= 0 && key < 255) {
            keyDown[key] = false;
        }
    }

    /**
     * Initialise the GUI
     */
    public void setup() {

        keyDown = new boolean[255];
        newKeyDown = new boolean[255];
        for (int i = 0; i < keyDown.length; i++) {
            keyDown[i] = false;
            newKeyDown[i] = false;
        }
        System.out.println("Setting up graphics, performing initialization");
        size(500, 500);
        frameRate(60);
        initDemo();
        smooth();
        transX = width / 2.0f;
        transY = height / 2.0f;
        pmousePressed = false;
        settings = new TestSettings();

        addMouseWheelListener(new MouseWheelListener() {
            public void mouseWheelMoved(MouseWheelEvent e) {
                int notches = e.getWheelRotation();
                Vec2 oldCenter = screenToWorld(width / 2.0f, height / 2.0f);
                if (notches < 0) {
                    scaleFactor = min(500f, scaleFactor * 1.05f);
                }
                else if (notches > 0) {
                    scaleFactor = max(.05f, scaleFactor / 1.05f);
                }
                Vec2 newCenter = screenToWorld(width / 2.0f, height / 2.0f);
                transX -= (oldCenter.x - newCenter.x) * scaleFactor;
                transY += (oldCenter.y - newCenter.y) * scaleFactor;
            }
        });
        
        postSetup();

    }

    static public int debugCount;
    
    public void postSetup() {
        
    }

    public void draw() {
        background(255);
        translate(transX, transY);
        scale(scaleFactor, -scaleFactor);
        strokeWeight(1.2f / scaleFactor);

        //Handle mouse dragging stuff
        //Left mouse attaches mouse joint to object.
        //Right mouse drags canvas.
        mouseWorld = screenToWorld(mouseX, mouseY);
        if (mouseButton == LEFT) {
            if (mousePressed && !pmousePressed) {
                MouseDown(mouseWorld);
            }
            else if (mousePressed) {
                if (mouseX != pmouseX || mouseY != pmouseY)
                    MouseMove(mouseWorld);
            }
            else if (pmousePressed) {
                MouseUp();
            }
        }
        else if (mouseButton == RIGHT) {
            if (mousePressed) {
                transX += mouseX - pmouseX;
                transY += mouseY - pmouseY;
            }
        }

        checkKeys();

        float initFollowX = 0f;
        float initFollowY = 0f;
        if (followedBody != null) {
            Vec2 a = worldToScreen(followedBody.m_position.x,followedBody.m_position.y);
            initFollowX = a.x;
            initFollowY = a.y;
        }
        
        preStep();
        
        debugCount = 0;
        // update data model
        //println("prestep");
        Step(settings);
        //println("poststep");
        
        postStep();
        
        if (followedBody != null) {
            Vec2 a = worldToScreen(followedBody.m_position.x,followedBody.m_position.y);
            transX += initFollowX - a.x;
            transY += initFollowY - a.y;
        }

        if (keyDown['d']) {
            System.out.println(debugCount);
        }

        if (needsReset) {
            // XXX m_world.clear();
            initDemo();
            needsReset = false;
        }
        for (int i = 0; i < newKeyDown.length; i++) {
            newKeyDown[i] = false;
        }
        pmousePressed = mousePressed;
    }

    /**
     * Stub method - override to add behavior per-frame.
     * Called immediately before physics step - keyboard
     * is already checked when called.
     */
    protected void preStep() {
    }
    
    /**
     * Called immediately after physics step.
     */
    protected void postStep() {
        
    }
    
    /**
     * Sets the camera to pan along with a body.
     * Still allows user to set relative camera position and scale.
     * @param b
     */
    public void setFollowed(Body b){
        followedBody = b;
    }

    protected Vec2 screenToWorld(float x, float y) {
        float wX = map(x - (transX - width / 2.0f), 0f, width, -width
                / (2.0f * scaleFactor), width / (2.0f * scaleFactor));
        float wY = map(y - (transY - height / 2.0f), height, 0f, -height
                / (2.0f * scaleFactor), height / (2.0f * scaleFactor));
        return new Vec2(wX, wY);
    }
    
    protected Vec2 worldToScreen(float x, float y) {
        float wX = map(x, -width / (2.0f * scaleFactor), width / (2.0f * scaleFactor),
                0f, width) + (transX - width / 2.0f);
        float wY = map(y, -height / (2.0f * scaleFactor), height / (2.0f * scaleFactor),
                height, 0f) + (transY - height / 2.0f);
        return new Vec2(wX, wY);
    }

    protected void checkKeys() {
        // override if needed
    }

    /**
     * Initialise the demo - clear the world
     */
    public final void initDemo() {
        //XXX m_world.clear();
        setupWorld();
        System.out.println("Initialising:" + getTitle());
        init(m_world);
    }

    public void setupWorld() {
        m_world = new World(new AABB(new Vec2(-100f, -100f), new Vec2(100f,
                100f)), new Vec2(0.0f, -10.0f), true);
        m_bomb = null;
        m_mouseJoint = null;
        boundImages.clear();
        followedBody = null;
    }

    /**
     * go(World) should be implemented by the demo, add the bodies/joints to the world.
     * 
     * @param world
     *            The world in which the simulation is going to run
     */
    protected void init(World world) {
        this.go(world);
    }

    public abstract void go(World w);
}