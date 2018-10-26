package entities;

import org.lwjgl.util.vector.Vector3f;

public class Entity {

	private Model model;
	private Vector3f position;

	private float heading, pitch, roll;
	private float scale;
	
	public Entity(Model model, Vector3f position) {
		this.model = model;
		this.position = position;
	}
	
	public Entity(Model model, Vector3f position, float heading, float pitch, float roll, float scale) {
		this.model = model;
		this.position = position;
		this.heading = heading;
		this.pitch = pitch;
		this.roll = roll;
		this.scale = scale;
	}
	
	public void increasePosition(float dx, float dy, float dz) {
		this.position.x += dx;
		this.position.y += dy;
		this.position.z += dz;
	}

	public Model getModel() {
		return model;
	}

	public Vector3f getPosition() {
		return position;
	}
	
	public float getHeading() {
		return heading;
	}

	public float getPitch() {
		return pitch;
	}

	public float getRoll() {
		return roll;
	}
	
	public float getScale() {
		return scale;
	}
	
	public void increaseRotation(float dheading, float dpitch, float droll) {
		this.heading += dheading;
		this.pitch += dpitch;
		this.roll += droll;
	}
	
	protected void setPosition(Vector3f position) {
		this.position = position;
	}
}
