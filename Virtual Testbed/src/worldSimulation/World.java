package worldSimulation;

import java.util.List;

import org.lwjgl.util.vector.Vector3f;

import entities.Cube;
import entities.Drone;

public class World {
	private Path path = new Path();
	private Drone drone;
	
	public World(Drone drone) {
		this.drone = drone;
	}
	
	public void addCube(Cube cube) {
		path.addCube(cube);
	}
	
	public List<Cube> getCubes() {
		return path.getCubes();
	}
	
	public boolean targetReached() {
		if(getCubes().isEmpty()) return true;
		int reached = -1;
		for(int i=0;i<getCubes().size();i++){
			Vector3f distance = Vector3f.sub(getCubes().get(i).getPosition(), drone.getPosition(), null);
			if(distance.length() < 4) reached = i;
		}
		if(reached!=-1) {
			getCubes().remove(reached);
		}
		return false;
	}
	
	public void reset(){
		getCubes().clear();
	}
	
	public void setPath(Path path) {
		this.path = path.copy();
	}
	
	public Path getPath() {
		return path.copy();
	}
	
	public void setDrone(Drone drone) {
		this.drone = drone;
	}
	
	public Cube searchCube(Vector3f position, boolean fromTop) {
		for (Cube cube: path.getCubes()) {
			Vector3f cubePos = cube.getPosition();
			if(fromTop){
				if((Math.abs(position.x - cubePos.x )< 0.5f)
						&&(Math.abs(position.z - cubePos.z )< 0.5f))
					return cube;
			} else {
				if((Math.abs(position.y - cubePos.y )< 0.5f)
						&&(Math.abs(position.z - cubePos.z )< 0.5f))
					return cube;
			}
		}	
		return null;
	}
}
