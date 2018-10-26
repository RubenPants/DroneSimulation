package control;

import java.util.ArrayList;
import java.util.List;

import entities.Cube;
import entities.Drone;
import worldSimulation.Path;
import worldSimulation.World;

public class DataController {

	private List<Path> importedPaths = new ArrayList<>();
	private Drone drone;
	private World world;
	
	public DataController() {
		drone = new Drone(1,1,1,1,1,1,1,1,1);
		world = new World(drone);
	}
	
	public void setWorldPath(int index) {
		world.setPath(importedPaths.get(index));
	}
	
	public int getPathsSize() {
		return importedPaths.size();
	}
	
	public World getWorld() {
		return world;
	}
	
	public void createNewPath() {
		importedPaths.add(new Path());
	}
	
	public void addPath(Path path){
		importedPaths.add(path);
	}
	
	public void removePath(int index) {
		importedPaths.remove(index);
	}
	
	public void removeCube(int cubeIndex, int pathIndex) {
		importedPaths.get(pathIndex).removeCube(cubeIndex);
	}
	
	public List<Path> getImportedPaths() {
		return importedPaths;
	}
	
	public void addCube(Cube cube, int pathNb) {
		importedPaths.get(pathNb).addCube(cube);
	}
}
