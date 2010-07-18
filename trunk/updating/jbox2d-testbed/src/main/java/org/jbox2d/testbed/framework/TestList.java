/**
 * Created at 5:34:33 PM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

import java.util.ArrayList;

import org.jbox2d.testbed.tests.PyramidTest;

/**
 * @author Daniel Murphy
 */
public class TestList {
	public static final ArrayList<TestbedTest> tests = new ArrayList<TestbedTest>();
	static{
		tests.add(new PyramidTest());
	}
}
