package collision;

public class Proxy {
	int lowerBounds[], upperBounds[];
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

	boolean IsValid() {
		return overlapCount != BroadPhase.INVALID;
	}
}
