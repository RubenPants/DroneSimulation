package swing_components;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.border.Border;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.lwjgl.util.vector.Vector3f;

public class InputPanel extends JPanel {
	
	private SettingsPanel fovPanel;
	private SettingsPanel redPanel;
	private SettingsPanel greenPanel;
	private SettingsPanel bluePanel;
	private SettingsPanel timePanel;
	private CubeAdderPanel cubePanel;
	
	private InfoPanel infoPanel;
	
	private SettingsListener settingsListener;
	
	public InputPanel() {
		Dimension dim = getPreferredSize();
		dim.width = 500;
		setPreferredSize(dim);
		
		
		ChangeListener changeListener = new ChangeListener() {
			public void stateChanged(ChangeEvent e) {
				settingsListener.changeRenderSettings(new SettingsEvent(this,
								fovPanel.getValue(),
								redPanel.getValue(),
								greenPanel.getValue(),
								bluePanel.getValue()));
			}
		};
		
		fovPanel = new SettingsPanel("FOV: ", JSlider.HORIZONTAL, 0, 180, 120, 1);
		fovPanel.addChangeListener(changeListener);
		redPanel = new SettingsPanel("Red: ", JSlider.HORIZONTAL, 0, 100, 100, 0.01f);
		redPanel.addChangeListener(changeListener);
		greenPanel = new SettingsPanel("Green: ", JSlider.HORIZONTAL, 0, 100, 100, 0.01f);
		greenPanel.addChangeListener(changeListener);
		bluePanel = new SettingsPanel("Blue: ", JSlider.HORIZONTAL, 0, 100, 100, 0.01f);
		bluePanel.addChangeListener(changeListener);
		
		timePanel = new SettingsPanel("Time Percent (%): ", JSlider.HORIZONTAL, 20, 2000, 200, 0.01f);
		timePanel.addChangeListener(new ChangeListener() {
			public void stateChanged(ChangeEvent e) {
				settingsListener.setTime(timePanel.getValue());
			}
		});
		
		cubePanel = new CubeAdderPanel();
		
		infoPanel = new InfoPanel();
		
		Border innerBorder = BorderFactory.createTitledBorder("Testbed Settings");
		Border outerBorder = BorderFactory.createEmptyBorder(5, 5, 5, 5);
		setBorder(BorderFactory.createCompoundBorder(outerBorder, innerBorder));
		
		setLayout(new GridBagLayout());
		
		GridBagConstraints gc = new GridBagConstraints();
		
		gc.gridx = 0;
		gc.gridy = 0;
		gc.weightx = 1;
		gc.weighty = .1;
		gc.fill = GridBagConstraints.HORIZONTAL;
		
		add(fovPanel, gc);
		
		gc.gridy = 1;
		
		add(redPanel, gc);
		
		gc.gridy = 2;

		add(greenPanel, gc);
		
		gc.gridy = 3;
		
		add(bluePanel, gc);
		
		gc.gridy = 4;
		
		add(timePanel, gc);
		
		gc.gridy = 5;
		
		add(cubePanel, gc);
		
		gc.gridy = 6;
		gc.weighty = 2.0;
		gc.anchor = GridBagConstraints.FIRST_LINE_START;
		add(infoPanel, gc);
	}
	
	public void setSettingsListener(SettingsListener listener) {
		this.settingsListener = listener;
		cubePanel.addSettingsListener(listener);
	}
	
	public void updateOrientationLabels(float heading, float pitch, float roll) {
		infoPanel.updateOrientationLabels(heading, pitch, roll);
	}
	 
	public void updateVelocityLabels(Vector3f position, Vector3f velocity, Vector3f angularVelocity) {
		infoPanel.updateVelocityLabels(position, velocity, angularVelocity);
	}
	
	public void updateTime(float frameTime, float time) {
		infoPanel.updateTime(frameTime, time);
	}
}
