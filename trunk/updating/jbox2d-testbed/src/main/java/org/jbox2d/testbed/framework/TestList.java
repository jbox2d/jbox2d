/**
 * Created at 5:34:33 PM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

import java.util.ArrayList;

import org.jbox2d.testbed.tests.OneSidedTest;
import org.jbox2d.testbed.tests.PyramidTest;
import org.jbox2d.testbed.tests.VaryingFriction;

/**
 * @author Daniel Murphy
 */
public class TestList {
	public static final ArrayList<TestbedTest> tests = new ArrayList<TestbedTest>();
	static{
		tests.add(new VaryingFriction());
		tests.add(new PyramidTest());
		tests.add(new OneSidedTest());
	}
}
