package swing_components;

import java.util.List;

import javax.swing.JPopupMenu;
import javax.swing.JTabbedPane;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.lwjgl.util.vector.Vector3f;

import interfaces.AutopilotConfig;
import entities.Cube;
import worldSimulation.Path;

public class TabPane extends JTabbedPane{
	
	InputPanel inputPanel;
	ConfigPanel configPanel;
	PathPanel pathPanel;
	
	public TabPane() {
		inputPanel = new InputPanel();
		configPanel = new ConfigPanel();
		pathPanel = new PathPanel();
		
		addTab("Inputs", inputPanel);
		addTab("Configuration", configPanel);
		addTab("Configure Path", pathPanel);
		addChangeListener(new ChangeListener() {
			
			@Override
			public void stateChanged(ChangeEvent e) {
			}
		});
		
	}

	public void setSettingsListener(SettingsListener listener) {
		inputPanel.setSettingsListener(listener);
		configPanel.setSettingsListener(listener);
	}
	
	public void updateOrientationLabels(float heading, float pitch, float roll) {
		inputPanel.updateOrientationLabels(heading, pitch, roll);
	}
	
	public void updateVelocityLabels(Vector3f velocity, Vector3f angularVelocity) {
		inputPanel.updateVelocityLabels(velocity, angularVelocity);
	}
	
	public void addPathListener(PathListener listener) {
		pathPanel.addPathListener(listener);
	}
	
	public void setPathData(List<Path> paths) {
		pathPanel.setPathData(paths);
	}
	
	public void setCubesData(List<Cube> cubes) {
		pathPanel.setCubesData(cubes);
	}
	
	public void refreshTables() {
		pathPanel.refreshTables();
	}
	
	public AutopilotConfig getConfigs() {
		return configPanel.getConfig();
	}
	
	public void updateTime(float frameTime, float time) {
		inputPanel.updateTime(frameTime, time);
	}
}
