package worldSimulation;

import org.lwjgl.util.vector.Vector3f;

public class DroneStartSettings {
	private Vector3f position = new Vector3f(0,4.7f,0);	//world coo
	private Vector3f velocity = new Vector3f(0,0,0);	//world coo
	private Vector3f angularVelocity = new Vector3f(0,0,0); //world coo
	
	private float heading = 0; //links rechts gedraaid in radialen
	private float pitch = 0; //voor onder boven gedraaid in radialen
	private float roll = 0;// (float)Math.PI;	//rond eigen as links rechts gedraaid in radialen
	
	public DroneStartSettings(){}

	public Vector3f getPosition() {
		return position;
	}

	public void setPosition(Vector3f position) {
		this.position = position;
	}

	public Vector3f getVelocity() {
		return velocity;
	}

	public void setVelocity(Vector3f velocity) {
		this.velocity = velocity;
	}

	public Vector3f getAngularVelocity() {
		return angularVelocity;
	}

	public void setAngularVelocity(Vector3f angularVelocity) {
		this.angularVelocity = angularVelocity;
	}

	public float getHeading() {
		return heading;
	}

	public void setHeading(float heading) {
		this.heading = heading;
	}

	public float getPitch() {
		return pitch;
	}

	public void setPitch(float pitch) {
		this.pitch = pitch;
	}

	public float getRoll() {
		return roll;
	}

	public void setRoll(float roll) {
		this.roll = roll;
	}
}
