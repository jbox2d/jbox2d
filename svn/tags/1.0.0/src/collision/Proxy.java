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
        lowerBounds[0] = lowerBounds[1] = 0;
        upperBounds[0] = upperBounds[1] = 0;
        overlapCount = BroadPhase.INVALID;
        timeStamp = 0;
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
