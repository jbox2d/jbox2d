package org.jbox2d.pooling.stacks;

import org.jbox2d.dynamics.TimeStep;

public class TimeStepStack extends DynamicTLStack<TimeStep> {
	@Override
	protected TimeStep newObjectInstance() {
		return new TimeStep();
	}
}
