package common;

public class Settings {
    // Global tuning constants based on MKS units.

    // public static final float EPSILON = 0.0000000001f;

    /** A "close to zero" float epsilon value for use */
    public static final float EPSILON = 1.1920928955078125E-7f;

    public static final float pi = 3.14159265359f;

    // Define your unit system here. The default system is
    // meters-kilograms-seconds. For the tuning to work well,
    // your dynamic objects should be bigger than a pebble and smaller
    // than a house.
    public static final float lengthUnitsPerMeter = 1.0f;

    public static final float massUnitsPerKilogram = 1.0f;

    public static final float timeUnitsPerSecond = 1.0f;

    // Use this for pixels:
    // public static final float lengthUnitsPerMeter = 50.0f;

    // Global tuning constants based on MKS units.

    // Collision
    public static final int maxManifoldPoints = 2;

    public static final int maxShapesPerBody = 64;

    public static final int maxPolyVertices = 8;

    public static final int maxProxies = 512; // this must be a power of two

    public static final int maxPairs = 8 * maxProxies; // this must be a
                                                            // power of two

    // Dynamics
    public static final float linearSlop = 0.005f * lengthUnitsPerMeter; // 0.5
                                                                            // cm

    public static final float angularSlop = 2.0f / 180.0f * pi; // 2 degrees

    public static final float velocityThreshold = 1.0f * lengthUnitsPerMeter
            / timeUnitsPerSecond; // 1 m/s

    public static final float maxLinearCorrection = 0.2f * lengthUnitsPerMeter; // 20
                                                                                // cm

    public static final float maxAngularCorrection = 8.0f / 180.0f * pi; // 8
                                                                            // degrees

    public static final float contactBaumgarte = 0.2f;

    // Sleep
    public static final float timeToSleep = 0.5f * timeUnitsPerSecond; // half
                                                                        // a
                                                                        // second

    public static final float linearSleepTolerance = 0.01f
            * lengthUnitsPerMeter / timeUnitsPerSecond; // 1 cm/s

    public static final float angularSleepTolerance = 0.5f / 180.0f / timeUnitsPerSecond; // 0.5 degrees/s

}
