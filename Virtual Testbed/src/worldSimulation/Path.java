package worldSimulation;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.util.vector.Vector3f;

import entities.Cube;
import tools.Tools;

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
	
	public interfaces.Path getAutopilotPath(){
		float[] xValues = new float[this.cubes.size()];
		float[] yValues = new float[this.cubes.size()];
		float[] zValues = new float[this.cubes.size()];
		for(int i=0;i<cubes.size();i++){
			Vector3f realPosition = cubes.get(i).getPosition();
			Vector3f randomVector = ((new Vector3f((float)Math.random(), (float)Math.random(), (float)Math.random()).normalise(null)));
			float scale = (float)(5*Math.random());
			Vector3f random5Vector = new Vector3f(scale*randomVector.x, scale*randomVector.y,scale*randomVector.z);
			
			Vector3f almostPosition = Tools.addVectors(realPosition, random5Vector);
			
			xValues[i] = realPosition.x;
			yValues[i] = realPosition.y;
			zValues[i] = realPosition.z;
		}
		return new interfaces.Path() {
			public float[] getX() {
				return xValues;
			}
			public float[] getY() {
				return yValues;
			}
			public float[] getZ() {
				return zValues;
			}
		};
		
	}
}
