package org.jbox2d.testbed.timingTests;

import org.jbox2d.collision.shapes.CircleDef;
import org.jbox2d.collision.shapes.PolygonDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.testbed.timingTests.SimpleTest;

/**
 * Benchmark - piston example (constantly bumping a bunch of
 * circles and boxes).  Should be a decent mix of circle and
 * polygon collisions/contacts, though very little joint work.
 * 
 * Rev 129 performance summary (details below class definition in source code):
 *
 *	No bullets:
 *	1.6 Average FPS: 			390.21332
 *	1.6 -server Average FPS: 	470.05365
 *
 *  (131+: with 1024M heap, 1.6 -server: 578.7675 FPS!)
 *
 *	All bullets:
 *	1.6 Average FPS: 			185.98808
 *	1.6 -server Average FPS: 	221.55266
 *
 *
 *  (C++ performance for no bullets is ~708 FPS, for comparison's sake)
 * @author eric
 *
 */
public class PistonBenchmark implements SimpleTest{
	public static boolean BULLETS = false;
	public RevoluteJoint m_joint1;
	public PrismaticJoint m_joint2;
	
	public void create(World world) {
    	{
			// Define crank.
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(0.5f, 2.0f);
			sd.density = 1.0f;

			RevoluteJointDef rjd = new RevoluteJointDef();

			Body prevBody = world.getGroundBody();

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, 7.0f);
			Body body = world.createBody(bd);
			body.createShape(sd);
			body.setMassFromShapes();

			rjd.initialize(prevBody, body, new Vec2(0.0f, 5.0f));
			rjd.motorSpeed = 1.0f * 3.1415f;
			rjd.maxMotorTorque = Float.MAX_VALUE;
			rjd.enableMotor = true;
			m_joint1 = (RevoluteJoint)world.createJoint(rjd);

			prevBody = body;

			// Define follower.
			sd.setAsBox(0.5f, 4.0f);
			bd.position.set(0.0f, 13.0f);
			body = world.createBody(bd);
			body.createShape(sd);
			body.setMassFromShapes();

			rjd.initialize(prevBody, body, new Vec2(0.0f, 9.0f));
			rjd.enableMotor = false;
			world.createJoint(rjd);

			prevBody = body;

			// Define piston
			sd.setAsBox(5.0f, 1.5f);
			bd.position.set(0.0f, 17.0f);
			body = world.createBody(bd);
			body.createShape(sd);
			body.setMassFromShapes();

			rjd.initialize(prevBody, body, new Vec2(0.0f, 17.0f));
			world.createJoint(rjd);

			PrismaticJointDef pjd = new PrismaticJointDef();
			pjd.initialize(world.getGroundBody(), body, new Vec2(0.0f, 17.0f), new Vec2(0.0f, 1.0f));

			//pjd.maxMotorForce = Float.MAX_VALUE;
			pjd.enableMotor = false;

			m_joint2 = (PrismaticJoint)world.createJoint(pjd);

			// Create a payload
			for (int i=0; i<100; ++i) {
				sd.setAsBox(0.4f,0.3f);
				sd.density = 0.1f;
				bd.position.set(-1.0f, 23.0f + i);
				if (BULLETS) bd.isBullet = true;
				else bd.isBullet = false;
				body = world.createBody(bd);
				body.createShape(sd);
				body.setMassFromShapes();
			}
			
			CircleDef cd = new CircleDef();
			cd.density = 2.0f;
			cd.radius = 0.36f;
			for (int i=0; i<100; ++i) {
				bd.position.set(1.0f, 23.0f + i);
				if (BULLETS) bd.isBullet = true;
				else bd.isBullet = false;
				body = world.createBody(bd);
				body.createShape(cd);
				body.setMassFromShapes();
			}
			
			sd.density = 0.0f;
			sd.friction = 0.0f;
			sd.setAsBox(1.0f, 100.0f);
			bd = new BodyDef();
			bd.position.set(-6.1f,50.0f);
			Body bod = world.createBody(bd);
			bod.createShape(sd);
			bd.position.set(6.1f,50.0f);
			bod = world.createBody(bd);
			bod.createShape(sd);
		}
	}
	
	public String toString() {
		return "Piston test";
	}
	
}

/*

Results for rev 129 of JBox2d

Summary:

No bullets:
1.6 Average FPS: 			390.21332
1.6 -server Average FPS: 	470.05365

All bullets:
1.6 Average FPS: 			185.98808
1.6 -server Average FPS: 	221.55266


Detailed results:

Mac OS 10.5.3 Java 6 (no -server)
Timing Piston test, no bullets
1000 frames per test, 20 tests.
3708994000 - 269.6148874870113 FPS
2664132000 - 375.3567766161737 FPS
2660371000 - 375.8874232202952 FPS
2653069000 - 376.92197225175823 FPS
2619859000 - 381.6999311795024 FPS
2614510000 - 382.4808472715729 FPS
2598526000 - 384.8335556388506 FPS
2711567000 - 368.79044478709176 FPS
2672973000 - 374.11526416465864 FPS
2456531000 - 407.07811136924386 FPS
2526658000 - 395.7797216718686 FPS
2435705000 - 410.5587499307182 FPS
2462603000 - 406.074385518088 FPS
2406905000 - 415.47132105338596 FPS
2435728000 - 410.554873122122 FPS
2403179000 - 416.1154870278077 FPS
2451098000 - 407.98042346736037 FPS
2403193000 - 416.1130629125501 FPS
2423926000 - 412.5538485910874 FPS
2402197000 - 416.28559189775024 FPS
Average time: 2.58558618E9
Average FPS: 390.21332

Mac OS 10.5.3 Java 6 (yes -server)
Timing Piston test, no bullets
1000 frames per test, 20 tests.
3076858000 - 325.00687389538285 FPS
2151497000 - 464.7926536732331 FPS
2139062000 - 467.494630824165 FPS
2111970000 - 473.49157421743683 FPS
2097921000 - 476.6623719386955 FPS
2125404000 - 470.49878517213665 FPS
2065729000 - 484.0906043338695 FPS
2110448000 - 473.8330439792878 FPS
2091541000 - 478.11637448178163 FPS
2120517000 - 471.58310921346066 FPS
2114015000 - 473.0335404431851 FPS
2067969000 - 483.5662430142811 FPS
2080364000 - 480.6851108748277 FPS
2087714000 - 478.9928122338596 FPS
2068451000 - 483.4535601761898 FPS
2090936000 - 478.25471463497684 FPS
2061044000 - 485.19100028917387 FPS
2065390000 - 484.17005989183644 FPS
2070758000 - 482.9149519161582 FPS
2060829000 - 485.24161878544993 FPS
Average time: 2.14292096E9
Average FPS: 470.05365

Mac OS 10.5.3 Java 6 (no -server)
Timing Piston test, all bullets
1000 frames per test, 20 tests.
6689210000 - 149.49448440099803 FPS
5501748000 - 181.76041505354297 FPS
5488888000 - 182.18626432166224 FPS
5363667000 - 186.43961304831188 FPS
5394727000 - 185.36619183880853 FPS
5433198000 - 184.05366415875145 FPS
5431069000 - 184.12581390514464 FPS
5418629000 - 184.54852694288536 FPS
5308725000 - 188.36914701741003 FPS
5354922000 - 186.74408329383695 FPS
5276919000 - 189.5045195880399 FPS
5300071000 - 188.67671772698895 FPS
5266092000 - 189.8941378160503 FPS
5259694000 - 190.12512895236873 FPS
5287822000 - 189.11377879210002 FPS
5282698000 - 189.29721138706017 FPS
5265863000 - 189.90239586559696 FPS
5169470000 - 193.44342843657088 FPS
5183957000 - 192.9028346492843 FPS
5159603000 - 193.81336122178394 FPS
Average time: 5.3918484E9
Average FPS: 185.98808

Mac OS 10.5.3 Java 6 (yes -server)
Timing Piston test, all bullets
1000 frames per test, 20 tests.
5671406000 - 176.32311987538893 FPS
4500211000 - 222.21180295768352 FPS
4520957000 - 221.19210600764396 FPS
4464306000 - 223.99898214862515 FPS
4474958000 - 223.4657844833404 FPS
4433345000 - 225.5633161867619 FPS
4472031000 - 223.61204562311843 FPS
4486088000 - 222.91136509136692 FPS
4489109000 - 222.761354201914 FPS
4483148000 - 223.05754795514224 FPS
4462389000 - 224.09520998729604 FPS
4468737000 - 223.7768747634958 FPS
4446194000 - 224.9114636023529 FPS
4479507000 - 223.23885195402084 FPS
4434062000 - 225.52684197920553 FPS
4469898000 - 223.718751524084 FPS
4420779000 - 226.20447663183347 FPS
4418487000 - 226.32181558981617 FPS
4464786000 - 223.97490047675296 FPS
4460567000 - 224.1867457657289 FPS
Average time: 4.5260483E9
Average FPS: 221.55266

*/

