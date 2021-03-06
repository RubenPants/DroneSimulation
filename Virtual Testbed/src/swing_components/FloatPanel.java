package swing_components;

import javax.swing.JLabel;
import static tools.Tools.*;

public class FloatPanel extends JLabel {
	
	String name;
	
	public FloatPanel(String name, float value) {
		super();
		this.name = name;
		setText(name + ": " + round(Math.toDegrees(value),2));
	}
	
	public void updateValueToDegrees(float value) {
		setText(name + ": " + round(Math.toDegrees(value),2));
	}
	
	public void updateValue(float value) {
		setText(name + ": " + round(value,2));
	}
}
