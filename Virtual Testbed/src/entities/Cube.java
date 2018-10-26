package entities;

import java.awt.Color;

import org.lwjgl.util.vector.Vector3f;

public class Cube extends Entity {

	private float width,height,length;
	private float hue,saturation;
	private Color color;
	
	
	public static final int[] CUBE_INDICES = {
			0,1,3,	
			3,1,2,	
			4,5,7,
			7,5,6,
			8,9,11,
			11,9,10,
			12,13,15,
			15,13,14,	
			16,17,19,
			19,17,18,
			20,21,23,
			23,21,22
	};
	
	public static final float[] CUBE_COLORVALUES = {
			.7f, .7f, .7f, .7f,
			.45f, .45f, .45f, .45f,
			.30f, .30f, .30f, .30f,
			.85f, .85f, .85f, .85f,
			1.00f, 1.00f, 1.00f, 1.00f,
			.15f, .15f, .15f, .15f
	};
	
	public Cube(Model cubeModel,float hue, float saturation, Vector3f position) {
		super(cubeModel, position);
		this.hue = hue;
		this.saturation = saturation;
		this.color = Color.getHSBColor(hue, saturation, 1);
	}
	
	public Cube(Model cubeModel, Vector3f position) {
		super(cubeModel, position);
		this.hue = (float)Math.random();
		this.saturation = (float) (1-Math.random()*0.7);
		this.color = Color.getHSBColor(hue, saturation, 1);
	}
	
	public float getHue() {
		return hue;
	}
	
	public float getSaturation() {
		return saturation;
	}
	
	public Vector3f getRGB() {
		return new Vector3f(color.getRed()/255.0f,color.getGreen()/255.0f,color.getBlue()/255.0f);
	}
	
	public void setColor(float hue, float saturation) {
		this.hue = hue;
		this.saturation = saturation;
		this.color = Color.getHSBColor(hue, saturation, 1);
	}
	
	public void setPosition(Vector3f position) {
		super.setPosition(position);
	}
}