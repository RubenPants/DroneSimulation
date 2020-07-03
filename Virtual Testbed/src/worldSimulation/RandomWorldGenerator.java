package worldSimulation;

import java.util.ArrayList;
import java.util.Random;

import org.lwjgl.util.vector.Vector3f;

import entities.Cube;
import entities.Model;

public class RandomWorldGenerator {

	private static float getDistance(Vector3f first, Vector3f second) {
		return (float) Math.sqrt(
				(Math.pow(first.x - second.x, 2) + Math.pow(first.y - second.y, 2) + Math.pow(first.z - second.z, 2)));
	}

	public static Path generateRandomWorld(int n, WorldGenerationMode mode, Model cubeModel) {
		ArrayList<Cube> cubes = new ArrayList<Cube>();
		for (int i = 0; i < n; i++) {
			Vector3f newCoord = generateCoord(i, mode);
			boolean toClose = false;
			for (Cube cube : cubes) {
				if (getDistance(cube.getPosition(), newCoord) < 200)
					toClose = true;
			}
			if (toClose) {
				System.out.println("TOO CLOSE");
				i--;
			} else {
				Cube newCube = new Cube(cubeModel, newCoord);
				cubes.add(newCube);
			}
		}
		Path randomPath = new Path(cubes);
		randomPath.setName("Random Path");
		return randomPath;
	}

	private static Vector3f generateCoord(int index, WorldGenerationMode mode) {
		ArrayList<Float> coordinates = randomCoordinate(mode);
		//return new Vector3f(coordinates.get(0), coordinates.get(1), coordinates.get(2));
		return new Vector3f( (int)(coordinates.get(0)/10)*10, (int)(coordinates.get(1)/10)*10, (int)(coordinates.get(2)/10)*10);
	}

	private static ArrayList<Float> randomCoordinate(WorldGenerationMode mode) {
		float xcoordinate = 0;
		float ycoordinate = 0;
		float zcoordinate = 0;
		ArrayList<Float> result = new ArrayList<Float>();
		switch (mode) {
		case FINDTILCORRECT:
			xcoordinate = (float) (Math.random() * 2000 - 1000);
			ycoordinate = (float) (Math.random() * 100 + 50);
			zcoordinate = (float) (Math.random() * -2000);
			break;
		case DISTANCEMODEL:
			float distance = (float) (Math.random() * 1500);
			int i = 0;
			while (i < 4 && distance < 200) {
				distance = distance * 2;
				i++;
			}
			float angle = (float) (float) (Math.random() * Math.PI * 2);

			xcoordinate = (float) (Math.cos(angle) * distance);
			ycoordinate = (float) (Math.sin(angle) * distance);
			break;
		case ARCHIMEDES:
			float r = 10;
			float t = (float) (2 * Math.PI * (float) Math.random());
			float u = (float) (Math.random() * 10) + (float) (Math.random() * 10);
			if (u > 10) {
				r = 20 - u;
			}
			xcoordinate = (float) (r * Math.cos(t));
			ycoordinate = (float) (r * Math.cos(t));
			break;
		default:
			break;
		}
		result.add(xcoordinate);
		result.add(ycoordinate);
		result.add(zcoordinate);
		return result;

	}
}
