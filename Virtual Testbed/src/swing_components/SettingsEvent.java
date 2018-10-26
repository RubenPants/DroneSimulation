package swing_components;

import java.util.EventObject;

public class SettingsEvent extends EventObject{

	private float fov;
	private float red;
	private float green;
	private float blue;
	
	public SettingsEvent(Object source) {
		super(source);
	}
	
	public SettingsEvent(Object source, float fov, float red, float green, float blue) {
		super(source);
		this.fov = fov;
		this.red = red;
		this.green = green;
		this.blue = blue;
	}

	public float getFov() {
		return fov;
	}

	public float getRed() {
		return red;
	}

	public float getGreen() {
		return green;
	}

	public float getBlue() {
		return blue;
	}
}
