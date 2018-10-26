package tools;

import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;
import org.lwjgl.util.vector.Vector2f;
import org.lwjgl.util.vector.Vector3f;

import entities.Camera;
import entities.Cube;
import worldSimulation.World;

public class CubeGrabber {
	
	private float width;
	private float height;
	
	private World world;
	
	private Vector3f positionTop;
	private Vector3f positionSide;
	private boolean inTopView = true;
	
	private Cube grabbedCube = null;
	
	public CubeGrabber(float width, float height, World world) {
		this.width = width;
		this.height = height;
		this.world = world;
	}

	public void update(Camera cameraTop, Camera cameraSide) {
		this.positionSide = cameraSide.getPosition();
		this.positionTop = cameraTop.getPosition();
		
		Vector3f mouseCoord = calculateMouseCoord();

		if(!Mouse.isButtonDown(0)) {
			grabbedCube = null;
		} else if(grabbedCube == null) {
			grabbedCube = world.searchCube(mouseCoord, inTopView);
		} else {
			Vector3f oldPos = grabbedCube.getPosition();
			if(inTopView)
				grabbedCube.setPosition(new Vector3f(mouseCoord.x,oldPos.y,mouseCoord.z));
			else 
				grabbedCube.setPosition(new Vector3f(oldPos.x,mouseCoord.y,mouseCoord.z));
		}
	}
	
	private Vector3f calculateMouseCoord() {
		float mouseX = Mouse.getX();
		float mouseY = Mouse.getY();
		Vector2f normalizedCoords = getNormalisedDeviceCoordinates(mouseX, mouseY);
		if(normalizedCoords.y > 0) {
			inTopView = true;
			normalizedCoords = new Vector2f(normalizedCoords.x,2*normalizedCoords.y - 1);
			return new Vector3f(positionTop.x - normalizedCoords.y*height/2,0,positionTop.z - normalizedCoords.x*width/2);
		} else {
			inTopView = false;
			normalizedCoords = new Vector2f(normalizedCoords.x,-2*normalizedCoords.y - 1);
			return new Vector3f(0,positionSide.y - normalizedCoords.y*height/2,positionSide.z - normalizedCoords.x*width/2);
		}
	}

	private Vector2f getNormalisedDeviceCoordinates(float mouseX, float mouseY) {
		float x = (2.0f * mouseX) / Display.getWidth() - 1f;
		float y = (2.0f * mouseY) / Display.getHeight() - 1f;
		return new Vector2f(x, y);
	}
}
