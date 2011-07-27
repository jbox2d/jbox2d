package org.jbox2d.serialization;

/**
 * Used to hear when an object is unsupported by the serializer or the deserializer.
 * This is so the de/serializer can keep going without throwing an exception and
 * stopping the entire thing
 * @author Daniel Murphy
 */
public interface UnsupportedListener {

	/**
	 * Called when an object is unsupported by the de/serializer.
	 * @param argException the exception describing the error
	 * @return if the process should stop and the exception be thrown
	 */
	public boolean isUnsupported(UnsupportedObjectException argException);
}
