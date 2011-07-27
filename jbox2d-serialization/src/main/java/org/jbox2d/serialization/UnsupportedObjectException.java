package org.jbox2d.serialization;

/**
 * Called when an object is unsupported by the serializer or deserializer. Pertains to shapes,
 * joints and other objects that might not be in some versions of the engine.
 * 
 * @author Daniel Murphy
 */
public class UnsupportedObjectException extends RuntimeException {

  private static final long serialVersionUID = 5915827472093183385L;

  public static enum Type {
    BODY, JOINT, SHAPE, OTHER
  }

  public Type type;

  public UnsupportedObjectException() {
    super();
  }

  public UnsupportedObjectException(String argMessage, Type argType) {
    super(argMessage);
    type = argType;
  }

  public UnsupportedObjectException(Throwable argThrowable) {
    super(argThrowable);
  }

  public UnsupportedObjectException(String argMessage, Throwable argThrowable) {
    super(argMessage, argThrowable);
  }

  @Override
  public String getLocalizedMessage() {
    return getMessage() + " [" + type + "]";
  }
}
