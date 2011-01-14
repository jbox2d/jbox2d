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
		tests.add(new DynamicTreeTest());
		tests.add(new Chain());
		tests.add(new Breakable());
		tests.add(new VerticalStack());
		tests.add(new PrismaticTest());
		tests.add(new BodyTypes());
		tests.add(new TheoJansen());
		tests.add(new BlobTest4());
		tests.add(new RevoluteTest());
		tests.add(new SliderCrankTest());
		tests.add(new ContinuousTest());
		tests.add(new CompoundShapesTest());
		tests.add(new ConfinedTest());
		tests.add(new DominoTest());
		tests.add(new MJWTest());
		tests.add(new VaryingFrictionTest());
		tests.add(new PyramidTest());
		tests.add(new OneSidedTest());
		tests.add(new DistanceTest());
		tests.add(new RayCastTest());
	}
}
