package dynamics.contacts;

import java.util.List;

import collision.Manifold;

public class NullContact extends Contact {

	public NullContact() {
		super(null, null);
	}

	@Override
	public void Evaluate() {
	}

	@Override
	public List<Manifold> GetManifolds() {
		return null;
	}
}
