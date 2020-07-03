package swing_components;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.BorderFactory;
import javax.swing.JPanel;
import javax.swing.border.Border;

import org.lwjgl.util.vector.Vector3f;

public class InfoPanel extends JPanel {
	private FloatPanel headingLabel;
	private FloatPanel pitchLabel;
	private FloatPanel rollLabel;
	
	private FloatPanel fpsLabel;
	private FloatPanel timeLabel;
	

	private Vector3Label positionLabel;
	private Vector3Label velocityLabel;
	private Vector3Label angularVelocityLabel;
	
	public InfoPanel() {
		Dimension dim = getPreferredSize();
		dim.width = 400;
		dim.height = 200;
		setPreferredSize(dim);
		
		fpsLabel = new FloatPanel("Frames Per Second", (float)Math.toRadians(100));
		timeLabel = new FloatPanel("Simulation Run Time", 0);
		
		headingLabel = new FloatPanel("Heading", 0);
		pitchLabel = new FloatPanel("Pitch", 0);
		rollLabel = new FloatPanel("Roll", 0);
		
		positionLabel = new Vector3Label("Position", new Vector3f(0,4.7f,0));
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
		
		add(positionLabel, gc);
		
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
	 
	public void updateVelocityLabels(Vector3f position, Vector3f velocity, Vector3f angularVelocity) {
		positionLabel.updateValue(position);
		velocityLabel.updateValue(velocity);
		angularVelocityLabel.updateValue(angularVelocity);
	}
	
	public void updateTime(float frameTime, float time) {
		fpsLabel.updateValue(1/frameTime);
		timeLabel.updateValue(time);
	}
}
