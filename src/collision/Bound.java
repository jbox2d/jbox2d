package collision;

public class Bound {
	int value;
	int proxyId;
	int stabbingCount;

	boolean IsLower() {
		return (value & 1) == 0;
	}

	boolean IsUpper() {
		return (value & 1) == 1;
	}
}
