package org.jbox2d.dynamics.contacts;

import org.jbox2d.structs.collision.ShapeType;
import org.jbox2d.structs.dynamics.contacts.ContactRegister;

/**
 * The class manages contact between two shapes. A contact exists for each overlapping
 * AABB in the broad-phase (except if filtered). Therefore a contact object may exist
 * that has no contact points.
 *
 * @author daniel
 */
public class Contact {
	// This contact should not participate in Solve
	// The contact equivalent of sensors
	public static final int e_sensorFlag		= 0x0001;
	// Generate TOI events
	public static final int e_continuousFlag	= 0x0002;
	// Used when crawling contact graph when forming islands.
	public static final int e_islandFlag		= 0x0004;
	// Used in SolveTOI to indicate the cached toi value is still valid.
	public static final int e_toiFlag			= 0x0008;
    // Set when the shapes are touching.
	public static final int e_touchingFlag		= 0x0010;
	// Disabled (by user)
	public static final int e_disabledFlag		= 0x0020;
	// This contact needs filtering because a fixture filter was changed.
	public static final int e_filterFlag		= 0x0040;
	
	public static final ContactRegister[][] s_registers = new ContactRegister[ShapeType.TYPE_COUNT][ShapeType.TYPE_COUNT];
	public static boolean s_initialized = false;
	
}
