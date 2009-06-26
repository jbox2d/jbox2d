package org.jbox2d.collision.structs;

public class Bound {
	public short value;
	public short proxyId;
	public short stabbingCount;
	
	public boolean isLower() {
		return (value & 1) == 0;
	}
	
	public boolean isUpper()  {
		return (value & 1) == 1;
	}
}
