package swing_components;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.BorderFactory;
import javax.swing.JPanel;
import javax.swing.border.Border;

import org.lwjgl.util.vector.Vector3f;

public class InfoPanel extends JPanel {
	private FloatLabel headingLabel;
	private FloatLabel pitchLabel;
	private FloatLabel rollLabel;
	
	private FloatLabel fpsLabel;
	private FloatLabel timeLabel;
	
	private Vector3Label velocityLabel;
	private Vector3Label angularVelocityLabel;
	
	public InfoPanel() {
		Dimension dim = getPreferredSize();
		dim.width = 400;
		dim.height = 200;
		setPreferredSize(dim);
		
		fpsLabel = new FloatLabel("Frames Per Second", (float)Math.toRadians(100));
		timeLabel = new FloatLabel("Simulation Run Time", 0);
		
		headingLabel = new FloatLabel("Heading", 0);
		pitchLabel = new FloatLabel("Pitch", 0);
		rollLabel = new FloatLabel("Roll", 0);
		
		velocityLabel = new Vector3Label("Velocity", new Vector3f(0,0,0));
		angularVelocityLabel = new Vector3Label("Relative Angular Velocity", new Vector3f(0,0,0));
		
		Border innerBorder = BorderFactory.createTitledBorder("Info");
		Border outerBorder = BorderFactory.createEmptyBorder(5, 5, 5, 5);
		setBorder(BorderFactory.createCompoundBorder(outerBorder, innerBorder));
		
		setLayout(new GridBagLayout());
		
		GridBagConstraints gc = new GridBagConstraints();
		
		gc.gridy = 1;
		gc.anchor = GridBagConstraints.BASELINE_TRAILING;
		add(fpsLabel, gc);
		
		gc.gridy++;
		
		add(timeLabel, gc);
		
		gc.gridy++;
		
		add(headingLabel, gc);
		
		gc.gridy++;
		
		add(pitchLabel, gc);
		
		gc.gridy++;
		
		add(rollLabel, gc);
		
		gc.gridy++;
		
		add(velocityLabel, gc);
		
		gc.gridy++;
		
		add(angularVelocityLabel, gc);
	}
	
	public void updateOrientationLabels(float heading, float pitch, float roll) {
		headingLabel.updateValueToDegrees(heading);
		pitchLabel.updateValueToDegrees(pitch);
		rollLabel.updateValueToDegrees(roll);
	}
	 
	public void updateVelocityLabels(Vector3f velocity, Vector3f angularVelocity) {
		velocityLabel.updateValue(velocity);
		angularVelocityLabel.updateValue(angularVelocity);
	}
	
	public void updateTime(float frameTime, float time) {
		fpsLabel.updateValue(1/frameTime);
		timeLabel.updateValue(time);
	}
}
