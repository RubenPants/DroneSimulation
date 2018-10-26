package swing_components;

import org.lwjgl.util.vector.Vector3f;

public interface PathListener {
	public void newPathCreated();
	public void generateRandomPath();
	public void pathDeleted(int index);
	public void cubeAdded(Vector3f position, int pathNb);
	public void cubeRemoved(int cubeIndex, int pathIndex);
	public void loadPathToWorld(int pathIndex);
}
