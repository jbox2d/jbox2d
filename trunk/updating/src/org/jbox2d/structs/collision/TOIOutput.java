package org.jbox2d.structs.collision;

/**
 * Output parameters for TimeOfImpact
 * @author daniel
 */
public class TOIOutput {
	public enum TOIOutputState {
		UNKNOWN, FAILED, OVERLAPPED, TOUCHING, SEPARATED
	}
	
	public TOIOutputState state;
	public float t;
}
