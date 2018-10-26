package entities;

import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import tools.Tools;

public class Camera {
	
	private Vector3f position;
	private float heading;
	private float pitch;
	private float roll;
	
	private float distanceFromDrone = -3;
	private float angleAroundDrone = 0;
	
	private Drone drone;
	private boolean unlocked;
	private boolean registredAlready = false;
	
	public Camera(Drone drone) {
		this.drone = drone;
		this.unlocked = true;
	}

	public Camera(Vector3f position, float heading, float pitch, float roll) {
		this.position = position;
		this.heading = heading;
		this.pitch = pitch;
		this.roll = roll;
		this.unlocked = true;
	}
	
	public void setDrone(Drone drone) {
		this.drone = drone;
	}
	
	public Vector3f getPosition() {
		if(drone!=null)
			return drone.getPosition();
		else
			return position;
	}

	public float getHeading() {
		if(drone!=null)
			return drone.getHeading();
		else
			return heading;
	}

	public float getPitch() {
		if(drone!=null)
			return drone.getPitch();
		else
			return pitch;
	}

	public float getRoll() {
		if(drone!=null)
			return drone.getRoll();
		else
			return roll;
	}
	
	public void move() {
		calculateLock();
		if(unlocked) {
			pitch -= calculatePitchChange();
			calculateHeading();
			calculatePosition();
		}
	}
	
	public void moveAroundDrone(Drone drone) {
		calculateLock();
		if(!unlocked) {
			calculateZoom();
			pitch += calculatePitchChange();
			calculateAngleAroundPlayer();
			float horizontalDistance = calculateHorizontalDistance();
			float verticalDistance = calculateVerticalDistance();
			calculateCameraPosition(horizontalDistance,verticalDistance, drone);
			this.heading = (drone.getHeading() + angleAroundDrone);
		}
	}
	
	private void calculatePosition() {
		Matrix4f matrix = Tools.createViewMatrixNoTranslate(this);
		Matrix4f.invert(matrix, matrix);
		
		if(Keyboard.isKeyDown(Keyboard.KEY_Z)) {
			Vector3f direction = Tools.scaleVector(Tools.transformVector(matrix, new Vector3f(0,1,0)), 0.1f);
			position = Tools.addVectors(position, direction);
		}
		if(Keyboard.isKeyDown(Keyboard.KEY_S)) {
			Vector3f direction = Tools.scaleVector(Tools.transformVector(matrix, new Vector3f(0,1,0)), -0.1f);
			position = Tools.addVectors(position, direction);
		}
		if(Keyboard.isKeyDown(Keyboard.KEY_Q)) {
			Vector3f direction = Tools.scaleVector(Tools.transformVector(matrix, new Vector3f(1,0,0)), -0.1f);
			position = Tools.addVectors(position, direction);
		}
		if(Keyboard.isKeyDown(Keyboard.KEY_D)) {
			Vector3f direction = Tools.scaleVector(Tools.transformVector(matrix, new Vector3f(1,0,0)), 0.1f);
			position = Tools.addVectors(position, direction);
		}
		int dWheel = Mouse.getDWheel();
		if(dWheel != 0) {
			Vector3f direction = Tools.scaleVector(Tools.transformVector(matrix, new Vector3f(0,0,1)), -dWheel*0.05f);
			position = Tools.addVectors(position, direction);
		}
	}
	
	private float calculatePitchChange() {
		if(Mouse.isButtonDown(0)) {
			return Mouse.getDY()*0.005f;
		} else {
			return 0;
		}
	}
	
	private void calculateHeading() {
		if(Mouse.isButtonDown(0)) {
			float headingChange = Mouse.getDX()*0.005f;
			heading += headingChange;
		}
	}
	
	private void calculateLock() {
		if(Keyboard.isKeyDown(Keyboard.KEY_SPACE)){
			if(!registredAlready) {
				unlocked = !unlocked;
				registredAlready = true;
				if(!unlocked) {
					heading = 0;
					pitch = -(float)Math.toRadians(20);
					angleAroundDrone = 0;
				}
			}
		} else {
			registredAlready = false;
		}
	}
	private void calculateCameraPosition(float horizDistance, float verticDistance, Drone drone) {
		float theta = drone.getHeading() + angleAroundDrone;
		float offsetX = (float) (horizDistance*Math.sin(theta));
		float offsetZ = (float) (horizDistance*Math.cos(theta));
		position.x = drone.getPosition().x - offsetX;
		position.z = drone.getPosition().z - offsetZ;
		position.y = drone.getPosition().y + verticDistance;
	}
	
	private float calculateHorizontalDistance() {
		return (float) (distanceFromDrone*Math.cos(pitch));
	}
	
	private float calculateVerticalDistance() {
		return (float) (distanceFromDrone*Math.sin(pitch));
	}
	
	private void calculateZoom() {
		float zoomLevel = Mouse.getDWheel()*0.005f;
		distanceFromDrone += zoomLevel;
	}
	private void calculateAngleAroundPlayer() {
		if(Mouse.isButtonDown(0)) {
			float angleChange = Mouse.getDX()*0.005f;
			angleAroundDrone -= angleChange;
		}
	}
}
