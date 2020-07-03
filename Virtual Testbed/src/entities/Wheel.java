package entities;

import org.lwjgl.util.vector.Vector3f;

public class Wheel {

	
	public static float RMAX=2500;
	
	private Vector3f position;
	private float tyreRadius;
	
	private float brakeForce;
	private boolean isPressed;
	private float pressed=0;
	
	public Wheel(Vector3f position, float tyreRadius){
		setPosition(position);
		this.tyreRadius = tyreRadius;
	}

	public Vector3f getPosition() {
		return new Vector3f(position.x,position.y,position.z);
	}

	public float getRadius() {
		return this.tyreRadius;
	}
	
	public float getBrakeForce() {
		return brakeForce;
	}

	public void setPosition(Vector3f position) {
		this.position = position;
	}

	public void setBrakeForce(float brakeForce) {
		if(brakeForce<0){
			this.brakeForce=0;
		}
		if(brakeForce<RMAX){
			this.brakeForce = brakeForce;
		}
	}
	
	public void setWheelPressed(boolean wheelPressed) {
		isPressed=wheelPressed;
		
	}

	public boolean isPressed() {
		
		return isPressed;
	}

	public float getPressed() {
		return pressed;
	}

	public void setPressed(float pressed) {
		this.pressed = pressed;
	}
}