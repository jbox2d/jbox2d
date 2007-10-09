package common;

public class Settings {
    // Global tuning constants based on MKS units.

    // public static final float EPSILON = 0.0000000001f;

    /** A "close to zero" float epsilon value for use */
    public static final float EPSILON = 1.1920928955078125E-7f;

    // Collision
    public static final int maxManifoldPoints = 2;

    public static final int maxShapesPerBody = 64;

    public static final int maxPolyVertices = 8;

    // this must be a power of two
    public static final int maxProxies = 512;

    // this must be a power of two
    public static final int maxPairs = 8 * maxProxies;

    // Dynamics
    public static final float linearSlop = 0.01f;

    public static final float angularSlop = (float) (2.0f / 180.0f * Math.PI);

    public static final float velocityThreshold = 1.0f;

    public static final float maxLinearCorrection = 0.2f;

    public static final float maxAngularCorrection = (float) (8.0f / 180.0f * Math.PI);

    public static final float contactBaumgarte = 0.2f;

    // Sleep
    public static final float timeToSleep = 0.5f;

    public static final float linearSleepTolerance = 0.01f;

    public static final float angularSleepTolerance = 0.01f;
}
