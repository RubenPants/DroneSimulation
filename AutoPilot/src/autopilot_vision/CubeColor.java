package autopilot_vision;

/**
 * A class of cube colors with a unique hue and saturation.
 * 
 * @author	Team Saffier
 * @version 	1.0
 */
public class CubeColor {

	/**
	 * Initialise this new cube color with given hue and saturation.
	 * 
	 * @param 	hue
	 * 			The hue for this new cube color.
	 * @param 	saturation
	 * 			The saturation for this new cube color.
	 */
	public CubeColor(float hue, float saturation) {
		this.hue = hue;
		this.saturation = saturation;
	}
	
	/**
	 * Get the hue of this cube color.
	 */
	public float getHue() {
		return hue;
	}
	
	/**
	 * Set the hue of this new color to match the given one.
	 * 
	 * @param 	hue
	 * 			The new hue for this cube color.
	 */
	public void setHue(float hue) {
		this.hue = hue;
	}
	
	/**
	 * Variable registering the hue of this cube color.
	 */
	private float hue;
	
	/**
	 * Get the saturation of this cube color.
	 */
	public float getSaturation() {
		return saturation;
	}
	
	/**
	 * Set the saturation of this new color to match the given one.
	 * 
	 * @param 	saturation
	 * 			The new saturation for this cube color.
	 */
	public void setSaturation(float saturation) {
		this.saturation = saturation;
	}
	
	/**
	 * Variable registering the saturation of this cube color.
	 */
	private float saturation;
	
}