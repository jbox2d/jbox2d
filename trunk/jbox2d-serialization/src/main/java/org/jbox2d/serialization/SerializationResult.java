package org.jbox2d.serialization;

import java.io.IOException;
import java.io.OutputStream;

/**
 * Container for holding serialization results.  Use
 * {@link #getValue()} to get the implementation-specific
 * result.
 * @author dmurph
 *
 */
public interface SerializationResult {
	
	/**
	 * The implementation-specific serialization
	 * result.
	 * @return serialization result
	 */
	public Object getValue();
	
	/**
	 * Writes the result to the given output stream.
	 * @param argOutputStream
	 * @throws IOException
	 */
	public void writeTo(OutputStream argOutputStream) throws IOException;
}
