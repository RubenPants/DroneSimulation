package swing_components;

import org.lwjgl.util.vector.Vector3f;

import interfaces.AutopilotConfig;
import worldSimulation.DroneStartSettings;

public interface SettingsListener {
	public void changeRenderSettings(SettingsEvent e);
	public void addCube(Vector3f position);
	public void setTime(float time);
	public void setConfigs(AutopilotConfig config);
	public void setStartSettigs(DroneStartSettings settings);
}
