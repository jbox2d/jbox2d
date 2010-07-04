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
package org.jbox2d.testbed.timingTests;

import org.jbox2d.collision.AABB;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.World;
import org.jbox2d.testbed.TestSettings;

/**
 * Benchmarking code.
 * 
 * @author ewjordan
 *
 */
public class TimingTest {
	
    private SimpleTest test;
    public int frames = 1000;
    public int iters = 40;
    protected TestSettings settings;
    protected World m_world;
    
    public TimingTest() {
    	test = new PistonBenchmark();
    }
    
    static public void main(String args[]) {
        TimingTest myTimingTest = new TimingTest();
        try {
        	myTimingTest.iters = Integer.parseInt(args[0]);
        } catch(Exception e) {
        	System.out.println("Suggested usage: java -jar -server -Xms1024M -Xmx2048M jbox2d-2.0.1-speedTest.jar (# of iters)\nFor example, java -jar -server -Xms1024M -Xmx2048M jbox2d-2.0.1-speedTest.jar 40");

        	System.exit(1);
        }
    	myTimingTest.go();
    }
    	
    public void go() {
    	
        settings = new TestSettings();
        long nanos = System.nanoTime();
        long diff = 0;
        System.out.println("Timing "+test);
        System.out.println(frames+" frames per test, "+iters+" tests.");
        System.out.println("Bullets are "+ ( (PistonBenchmark.BULLETS)?"on":"off") );
        long diffsum = 0;
        double fpssum = 0;
        for (int i=0; i<iters; i++){

            System.gc();
            
            nanos = System.nanoTime();
            
            setupWorld();
            test.create(m_world);
            
            
            nanos = System.nanoTime();
            for (int j=0; j<frames; j++) {
                Step(settings);
            }
            diff = System.nanoTime() - nanos;
            double fps = frames / ( diff / ((double)1000000000) );
            System.out.println("Test "+i+ " - "+ fps + " FPS");// + " ns for "+frames+" frames.");
            diffsum += diff;
            fpssum += fps;
        }
        
        float avdiff = diffsum / ((float)iters);
        float avfps = (float)fpssum / iters;
        System.out.println("Average time: "+avdiff);
        System.out.println("Average FPS: "+avfps);
        
    }

    void Step(TestSettings settings) {
        float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;
        m_world.step(timeStep, settings.iterationCount);
    }

    public void setupWorld() {
        m_world = new World(new AABB(new Vec2(-100f, -100f), new Vec2(100f,
                200f)), new Vec2(0.0f, -10.0f), true);
        m_world.setPositionCorrection(true);
        m_world.setWarmStarting(true);
    }
    
}

