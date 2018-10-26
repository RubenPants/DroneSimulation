package worldSimulation;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.util.vector.Vector3f;

import entities.Cube;

public class Path {

	private List<Cube> cubes = new ArrayList<>();
	private String name = null;
	
	public Path() {}
	
	public Path(List<Cube> positions) {
		this.cubes = positions;
	}
	
	public List<Cube> getCubes(){
		return cubes;
	}
	
	public void addCube(Cube cube) {
		if(colorsTooClose(cube))
			cube.setColor((float)Math.random(), (float) (1-Math.random()*0.7));
		cubes.add(cube);
	}
	
	public void removeCube(int index) {
		cubes.remove(index);
	}
	
	public Path copy() {
		List<Cube> cubesCopied = new ArrayList<>();
		for(Cube cube: getCubes()) {
			cubesCopied.add(new Cube(cube.getModel(),cube.getHue(),cube.getSaturation(),cube.getPosition()));
		}
		return new Path(cubesCopied);
	}
	
	public String getName() {
		return name;
	}
	
	public void setName(String name) {
		this.name = name;
	}
	
	public void controlColors() {
		for(Cube cube: getCubes()) {
			while(colorsTooClose(cube)) {
				cube.setColor((float)Math.random(), (float) (1-Math.random()*0.7));
			}
		}
	}
	
	private boolean colorsTooClose(Cube cube) {
		for(Cube cube2: getCubes()) {
			if(cube==cube2) continue;
			if (Math.abs(cube.getSaturation() - cube2.getSaturation()) < 0.05 &&
					Math.abs(cube.getHue() - cube2.getSaturation()) < 0.05) {
				return true;
			}
		}
		return false;
	}
}
