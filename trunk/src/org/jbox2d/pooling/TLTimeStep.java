package org.jbox2d.pooling;

import org.jbox2d.dynamics.TimeStep;

public class TLTimeStep extends ThreadLocal<TimeStep> {
	@Override
	protected TimeStep initialValue(){
		return new TimeStep();
	}
}
