package collision;

public class Proxy {
    public int lowerBounds[];

    public int upperBounds[];

    int overlapCount;

    int timeStamp;

    Object userData;

    public Proxy() {
        lowerBounds = new int[2];
        upperBounds = new int[2];
    }

    int GetNext() {
        return lowerBounds[0];
    }

    void SetNext(int next) {
        lowerBounds[0] = next;
    }

    public boolean IsValid() {
        return overlapCount != BroadPhase.INVALID;
    }
}
