/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
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


/*
 * TimingTest.java
 * 
 * Uses System.nanoTime() to time JBox2D performance.
 * 
 * Alter frames, iters, and testToTime.
 * 
 * Draws last frame of tests on top of one another to make sure test
 * has run correctly, more or less.
 * 
 * If there is a problem and you get huge frame rates, make sure that the test
 * you are running uses the passed "world" parameter instead of the member "m_world"
 * in its go(World world) method.
 * 
 * Also, be careful - if your simulation goes to sleep, it will run much faster.
 * This is probably not desired in a timing test - if things show up blue when drawn,
 * they have gone to sleep.
 * Make sure that something is actually happening for most of the simulation!
 * 
 * Sample results on my PowerBook G4 (1.5 GHz, 512 MB RAM)
 * Using Nov. 15 2007 SVN codebase
 * WASHING_MACHINE test
 * 2000 frames, 10 tests.
 * 37981407000 - 52.6573436313194 FPS
 * 38646444000 - 51.75120381062744 FPS
 * 37170935000 - 53.80548000743054 FPS
 * 40516547000 - 49.36254809670725 FPS
 * 39182865000 - 51.042719821534234 FPS
 * 38868422000 - 51.455652097221744 FPS
 * 37352766000 - 53.543558193254015 FPS
 * 38070922000 - 52.53353202215591 FPS
 * 39065642000 - 51.19588204898822 FPS
 * 38093416000 - 52.502511195110465 FPS
 * Average time: 3.8494937E10
 * Average FPS: 51.98504
 * 
 * 
 */
package org.jbox2d.testbed;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.CircleShape;
import org.jbox2d.collision.PolygonShape;
import org.jbox2d.collision.Shape;
import org.jbox2d.collision.ShapeType;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.World;
import org.jbox2d.testbed.tests.*;

import processing.core.PApplet;

/**
 * Currently inactive.
 * 
 * @author ewjordan
 *
 */
public class TimingTest extends PApplet {
	
//    
//    public int frames = 500;
//    public int iters = 100;
//    public TestSelection testToTime = TestSelection.CIRCLES;
//    
//    public enum TestSelection{
//        WASHING_MACHINE, COMPOUND_SHAPES, DOMINO, PYRAMID, CIRCLES, DOMINO_TOWER
//    }
//    
//    static public void main(String args[]) {
//        PApplet.main(new String[] { "org.jbox2d.testbed.TimingTest" });
//    }
//
//    protected TestSettings settings;
//
//    protected World m_world;
//
//    public TimingTest() {
//
//    }
//
//    void Step(TestSettings settings) {
//        float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;
//
//        World.ENABLE_WARM_STARTING = true;//settings.enableWarmStarting;
//        World.ENABLE_POSITION_CORRECTION = true;//settings.enablePositionCorrection;
//
//        m_world.step(timeStep, settings.iterationCount);
//
//        // m_world.m_broadPhase.Validate();
//    }
//
//    void drawFrame() {
//        float transX = width / 2.0f;
//        float transY = height / 2.0f;
//        pushMatrix();
//        translate(transX, transY);
//        scale(10f, -10f);
//        strokeWeight(1.2f / 10f);
//        for (Body b = m_world.m_bodyList; b != null; b = b.m_next) {
//            for (Shape s = b.m_shapeList; s != null; s = s.m_next) {
//                if (b.m_invMass == 0.0f) {
//                    DrawShape(s, color(100, 100, 100));
//                }
//                else if (b.isSleeping()) {
//                    DrawShape(s, color(30, 30, 90));
//                }
//                else {
//                    DrawShape(s, color(30, 30, 30));
//
//                }
//            }
//        }
//        popMatrix();
//    }
//    
//    void DrawShape(Shape shape, int c) {
//        //UPDATE FOR 2.0!!!
//    }
//
//
//    /**
//     * Initialise and run tests
//     */
//    public void setup() {
//        size(500, 500);
//        settings = new TestSettings();
//        long nanos = System.nanoTime();
//        long diff = 0;
//        System.out.println(testToTime + " test");
//        System.out.println(frames+" frames per test, "+iters+" tests.");
//        long diffsum = 0;
//        double fpssum = 0;
//        for (int i=0; i<iters; i++){
//            nanos = System.nanoTime();
//            setupWorld();
//            switch(testToTime){
//                case WASHING_MACHINE:
//                    WashingMachine wm = new WashingMachine();
//                    wm.go(m_world);
//                    break;
//                case COMPOUND_SHAPES:
//                    CompoundShapes cs = new CompoundShapes();
//                    cs.go(m_world);
//                    break;
//                case DOMINO:
//                    Domino d = new Domino();
//                    d.go(m_world);
//                    break;
//                case PYRAMID:
//                    Pyramid p = new Pyramid();
//                    p.go(m_world);
//                    break;
//                case CIRCLES:
//                    Circles c = new Circles();
//                    c.go(m_world);
//                    break;
//                case DOMINO_TOWER:
//                    DominoTower dp = new DominoTower();
//                    dp.go(m_world);
//                    break;
//            }
//            
//            //long initdiff = System.nanoTime() - nanos;
//            nanos = System.nanoTime();
//            for (int j=0; j<frames; j++) {
//                Step(settings);
//            }
//            diff = System.nanoTime() - nanos;
//            double fps = frames / ( diff / ((double)1000000000) );
//            System.out.println(diff + " - "+ fps + " FPS");// + " ns for "+frames+" frames.");
//            diffsum += diff;
//            fpssum += fps;
//            drawFrame();
//        }
//        
//        float avdiff = diffsum / ((float)iters);
//        float avfps = (float)fpssum / iters;
//        System.out.println("Average time: "+avdiff);
//        System.out.println("Average FPS: "+avfps);
//        
//
//    }
//
//    public void setupWorld() {
//        m_world = new World(new AABB(new Vec2(-100f, -100f), new Vec2(100f,
//                100f)), new Vec2(0.0f, -10.0f), true);
//    }
//    
}