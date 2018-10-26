package autopilot_vision;

import static java.lang.Math.toRadians;

import java.io.File;
import java.util.ArrayList;

import org.junit.Before;
import org.junit.Test;
import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import autopilot_vision.CubeDetectionAlgorithm.CubeDetectionAlgorithmType;
import junit.framework.TestCase;

/**
 * A class for testing distance calculation.
 * 
 * @author	Team Saffier
 * @version	1.0
 */
public class DistanceTests extends TestCase {

	/**
	 * Variable registering the URIs of test images for this test case.
	 */
	private ArrayList<String> testImages;
	
	/**
	 * Prepare the unit tests.
	 */
	@Before
	protected void setUp() throws Exception {
		
		super.setUp();
		
		// Initialise test image paths
		// ClassLoader classLoader = Thread.currentThread().getContextClassLoader();
		this.testImages = new ArrayList<String>();
//		this.testImages.add("Images/test/distance/dist10.bmp");
//		this.testImages.add("Images/test/distance/dist5.bmp");
//		this.testImages.add("Images/test/distance/dist3.bmp");
//		this.testImages.add("Images/test/distance/dist108.bmp");
//		this.testImages.add("Images/test/distance/dist52.bmp");
//		this.testImages.add("Images/test/distance/dist_new.bmp");
		
		this.testImages.add("Images/test/distance/dist-3-3--3.bmp");
		this.testImages.add("Images/test/distance/dist-3--3--3.bmp");
		this.testImages.add("Images/test/distance/dist3--3--3.bmp");
		this.testImages.add("Images/test/distance/dist3-3--3.bmp");
		this.testImages.add("Images/test/distance/dist0-0--10.bmp");
		this.testImages.add("Images/test/distance/dist0-0--3.bmp");
		
	}

	/**
	 * Test the distance calculations with some basic input images.
	 */
	@Test
	public void testDistanceCalculation_Basic() {
		
		CubeDetectionAlgorithm algorithm = CubeDetectionAlgorithm.initializeAlgorithm(CubeDetectionAlgorithmType.ADVANCED);
		float haov = (float)(2 * Math.PI / 3), vaov = (float)(2 * Math.PI / 3);
		int cols = 200, rows = 200;
		Vector3f cameraCoordinates = new Vector3f(0, 0, 0);
		Matrix4f cameraToWorldTransformationMatrix = getDroneToWorldTransformationMatrix(0, 0, 0);
		
		for (String filePath : this.testImages) {
			
			System.out.println("--- " + filePath + " ---");
			
			// long tic = System.nanoTime();
			Image image = Image.createImageUsingFile(new File(filePath));
			if (image == null)
				break;
			
			long tic = System.nanoTime();
			ArrayList<Cube> cubes = algorithm.locateUnitCubes(image);
			System.out.println("Performance locating cubes : " + ((System.nanoTime() - tic) / Math.pow(10, 6)) + "ms");
			for (Cube cube : cubes) {
				AutopilotImageAnalyser.updateDistanceToCube(cube, haov, cols, rows);
				Point3D location = AutopilotImageAnalyser.approximateLocation(
						cube, 
						cube.getDistance(), 
						cameraCoordinates, 
						cameraToWorldTransformationMatrix, 
						cols, 
						rows, 
						haov, 
						vaov);
				System.out.println("Cube found : " + cube + " (distance = " + cube.getDistance() + ", location = " + location + ")");
			}
			
		}
		
	}
	
	/**
	 * Calculate the drone-to-world transformation matrix for given heading, pitch
	 * 	and heading of the drone.
	 * 
	 * @param 	heading
	 * 			The heading of the drone.
	 * @param 	pitch
	 * 			The pitch of the drone.
	 * @param 	roll
	 * 			The roll of the drone.
	 * @return	The transformation matrix for transforming drone coordinates to world
	 * 			coordinates.
	 */
	private Matrix4f getDroneToWorldTransformationMatrix(float heading, float pitch, float roll) {
		Matrix4f matrix = new Matrix4f();
		matrix.setIdentity();
		Matrix4f.rotate((float)toRadians(heading), new Vector3f(0, 1, 0), matrix, matrix);
		Matrix4f.rotate((float)toRadians(pitch), new Vector3f(1, 0, 0), matrix, matrix);
		Matrix4f.rotate((float)toRadians(roll), new Vector3f(0, 0, 1), matrix, matrix);
		return matrix;
	}

}
