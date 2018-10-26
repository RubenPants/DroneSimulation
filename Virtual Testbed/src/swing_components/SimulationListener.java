package swing_components;

import rendering.CameraView;
import rendering.SimulationStatus;

public interface SimulationListener {
	public void swapSimulationStatus(SimulationStatus status);
	
	public void swapView(CameraView view);
	
	public void testingRequested();
}
