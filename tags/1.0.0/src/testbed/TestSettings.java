package testbed;

public class TestSettings {
    public float hz;

    public int iterationCount;

    public boolean drawStats;

    public boolean drawContacts;

    public boolean drawImpulses;

    public boolean drawAABBs;

    public boolean drawPairs;

    public boolean enableWarmStarting;

    public boolean enablePositionCorrection;

    public TestSettings() {
        hz = 60;
        iterationCount = 10;
        drawStats = false;
        drawContacts = false;
        drawImpulses = false;
        drawAABBs = false;
        drawPairs = false;
        enableWarmStarting = true;
        enablePositionCorrection = true;
    }
}
