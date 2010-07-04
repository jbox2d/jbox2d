package org.jbox2d.dynamics.contacts;

import org.jbox2d.common.Sweep;

public class NullContact extends Contact{

	@Override
	protected void evaluate() {
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public float computeTOI(Sweep sweepA, Sweep sweepB){
		return -1;
	}
}
