/**
 * Created at 5:34:33 PM Jul 17, 2010
 */
package org.jbox2d.testbed.tests;

import java.util.ArrayList;

import org.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class TestList {
	public static final ArrayList<TestbedTest> tests = new ArrayList<TestbedTest>();
	static{
		
		// general collision
		// watching...
		tests.add(new DominoTower());
		tests.add(new CircleStress());
		tests.add(new VaryingRestitution());
		tests.add(new VaryingFrictionTest());
		tests.add(new SphereStack());
		tests.add(new CompoundShapesTest());
		tests.add(new DominoTest());
		tests.add(new VerticalStack());
		tests.add(new PyramidTest());
		// more interactive..
		tests.add(new ShapeEditing());
		tests.add(new Breakable());
		tests.add(new OneSidedTest());
		tests.add(new PolyShapes());
		tests.add(new BodyTypes());
		tests.add(new CharacterCollision());
		tests.add(new ApplyForce());

		// processing/filtering
		tests.add(new CollisionFiltering());
		tests.add(new CollisionProcessing());
		tests.add(new SensorTest());

		// joints
		tests.add(new PrismaticTest());
		tests.add(new RevoluteTest());
		tests.add(new Pulleys());
		tests.add(new LineJointTest());
		tests.add(new Gears());
		tests.add(new Web());
		tests.add(new Chain());
		tests.add(new Cantilever());
		tests.add(new SliderCrankTest());
		tests.add(new BlobTest4());
		tests.add(new TheoJansen());
		
		// ccd
		tests.add(new ContinuousTest());
		tests.add(new ConfinedTest());
		
		// raycast
		tests.add(new RayCastTest());
		tests.add(new EdgeShapes());

		// misc
		tests.add(new DynamicTreeTest());
		tests.add(new DistanceTest());
	}
}
