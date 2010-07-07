/**
 * Created at 3:59:59 AM Jul 7, 2010
 */
package org.jbox2d.dynamics;

/**
 * The body type.
 * static: zero mass, zero velocity, may be manually moved
 * kinematic: zero mass, non-zero velocity set by user, moved by solver
 * dynamic: positive mass, non-zero velocity determined by forces, moved by solver
 * @author daniel
 */
public enum BodyType {
	STATIC, KINEMATIC, DYNAMIC
}
