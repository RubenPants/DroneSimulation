package worldSimulation;

import java.util.ArrayList;
import java.util.Random;

import org.lwjgl.util.vector.Vector3f;

import entities.Cube;
import entities.Model;

public class RandomWorldGenerator {
	
	public static Path generateRandomWorld(int n, WorldGenerationMode mode, Model cubeModel){
		ArrayList<Cube> cubes = new ArrayList<Cube>();
		for (int i = 0; i < n; i++) {
			Vector3f newCoord = generateCoord(i, mode);
			Cube newCube = new Cube(cubeModel, newCoord);
			cubes.add(newCube);
		}
		Path randomPath = new Path(cubes);
		randomPath.setName("Random Path");
		return randomPath; 
	}

	private static Vector3f generateCoord(int index, WorldGenerationMode mode) {
		float zcoordinate=-40 - 40*index;
		ArrayList<Float> coordinates = randomCoordinate(mode);
		float xcoordinate = coordinates.get(0);
		float ycoordinate = coordinates.get(1);
	
		Vector3f cubePos = new Vector3f(xcoordinate,ycoordinate,zcoordinate);
		return cubePos;
	}
	
	private static ArrayList<Float> randomCoordinate(WorldGenerationMode mode){
		float xcoordinate=10;
		float ycoordinate=10;
		ArrayList<Float> result=new ArrayList<Float>();
		switch(mode){
			case FINDTILCORRECT:
				while(Math.sqrt(Math.pow(xcoordinate,2)+Math.pow(ycoordinate,2))>10){
				xcoordinate=(float) (Math.random()*10);
				ycoordinate=(float) (Math.random()*10);
				break;
			}
			case DISTANCEMODEL:
				float distance=(float) (Math.random()*10);
				int i=0;
				while(i<4 && distance<5){
					distance=distance*2;
					i++;
				}
				float angle=(float) (float) (Math.random()*Math.PI*2);
				
				xcoordinate=(float) (Math.cos(angle)*distance);
				ycoordinate=(float) (Math.sin(angle)*distance);
				break;
			case ARCHIMEDES:
				float r=10;
				float t = (float)( 2*Math.PI*(float)Math.random());
				float u = (float) (Math.random()*10)+(float) (Math.random()*10);
				if(u>10){
					r=20-u;
				}
				xcoordinate=(float) (r*Math.cos(t));
				ycoordinate=(float) (r*Math.cos(t));
				break;
				
			default:
				break;	
	   		}
	   result.add(xcoordinate);
	   result.add(ycoordinate);
	   return result;
	   
		  
		  
   }
}
