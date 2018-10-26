package autopilot_vision;

import java.io.File;
import java.util.ArrayList;
import junit.framework.TestCase;
import org.junit.Before;
import org.junit.Test;

import autopilot_vision.CubeDetectionAlgorithm.CubeDetectionAlgorithmType;

/**
 * A class for testing cube detection algorithms.
 * 
 * @author	Team Saffier
 * @version	1.0
 */
public class CubeDetectionAlgorithmTests extends TestCase {
		
	/**
	 * Variable registering the URIs of test images for this test case.
	 */
	private ArrayList<String> testImages;
	
	/**
	 * Prepare the unit tests.
	 */
	@Before
	public void setUp() throws Exception {
		
		super.setUp();
		
		// Initialise test image paths
		// ClassLoader classLoader = Thread.currentThread().getContextClassLoader();
		this.testImages = new ArrayList<String>();
		this.testImages.add("Images/test/save1.bmp");
		this.testImages.add("Images/test/save2.bmp");
		this.testImages.add("Images/test/save3.bmp");
		this.testImages.add("Images/test/save4.bmp");
		this.testImages.add("Images/test/save5.bmp");
		this.testImages.add("Images/test/save6.bmp");
		this.testImages.add("Images/test/save7.bmp");
		this.testImages.add("Images/test/save8.bmp");
		this.testImages.add("Images/test/save9.bmp");
		this.testImages.add("Images/test/save10.bmp");
		this.testImages.add("Images/test/save11.bmp");
		this.testImages.add("Images/test/save12.bmp");

	}

	/**
	 * Test the image analyser with some basic input images.
	 */
	/*
	@Test
	public void testCubeLocalisation() {
		
		// Initialise image analyser
		ArrayList<CubeDetectionAlgorithm> algorithms = new ArrayList<CubeDetectionAlgorithm>();
		algorithms.add(CubeDetectionAlgorithm.initializeAlgorithm(CubeDetectionAlgorithmType.BASIC));
		// algorithms.add(CubeDetectionAlgorithm.initializeAlgorithm(CubeDetectionAlgorithmType.OPEN_CV));
		// algorithms.add(CubeDetectionAlgorithm.initializeAlgorithm(CubeDetectionAlgorithmType.OPEN_CL));
		// algorithms.add(CubeDetectionAlgorithm.initializeAlgorithm(CubeDetectionAlgorithmType.HOUGH));
		int index = 1;
		
		for (CubeDetectionAlgorithm algorithm : algorithms) {
			
			System.out.println("-------------------------------------------");
			System.out.println("           RED CUBE ALGORITHM (" + index++ + ")            ");
			System.out.println("-------------------------------------------");
			
			for (String filePath : this.testImages) {
				
				System.out.println("--- " + filePath + " ---");
				
				// long tic = System.nanoTime();
				Image image = Image.createImageUsingFile(new File(filePath));
				if (image == null)
					return;
				// float[] hsvBytes = ImageAnalyser.convertRGBToHSV(bytes);
				// System.out.println("Performance converting image to HSV array : " + (System.currentTimeMillis() - tic) + "ms");
				
				long tic = System.nanoTime();
				Cube cube = algorithm.locateUnitCube(image, 0.0f, 1.0f);
				System.out.println("Performance locating cube : " + ((System.nanoTime() - tic) / Math.pow(10, 6)) + "ms");
				if (cube != null)
					System.out.println("Center = " + cube.getCenter());
				else
					System.out.println("No cube was found.");
				
			}
			
		}
				
	}
	*/
	
	/**
	 * Test the image analyser with some basic input images.
	 * 	This looks for *all* cubes.
	 */
	@Test
	public void testMultiCubeLocalisation() {
		
		// Initialise image analyser
		ArrayList<CubeDetectionAlgorithm> algorithms = new ArrayList<CubeDetectionAlgorithm>();
		algorithms.add(CubeDetectionAlgorithm.initializeAlgorithm(CubeDetectionAlgorithmType.BASIC));
		algorithms.add(CubeDetectionAlgorithm.initializeAlgorithm(CubeDetectionAlgorithmType.ADVANCED));
		// algorithms.add(CubeDetectionAlgorithm.initializeAlgorithm(CubeDetectionAlgorithmType.OPEN_CL));
		// algorithms.add(CubeDetectionAlgorithm.initializeAlgorithm(CubeDetectionAlgorithmType.HOUGH));
		int index = 1;
		
		for (CubeDetectionAlgorithm algorithm : algorithms) {
			
			System.out.println("-------------------------------------------");
			System.out.println("           ALL CUBES ALGORITHM (" + index++ + ")                ");
			System.out.println("-------------------------------------------");
			
			for (String filePath : this.testImages) {
				
				System.out.println("--- " + filePath + " ---");
				
				// long tic = System.nanoTime();
				Image image = Image.createImageUsingFile(new File(filePath));
				if (image == null)
					return;
				// float[] hsvBytes = ImageAnalyser.convertRGBToHSV(bytes);
				// System.out.println("Performance converting image to HSV array : " + (System.currentTimeMillis() - tic) + "ms");
				
				long tic = System.nanoTime();
				ArrayList<Cube> cubes = algorithm.locateUnitCubes(image);
				System.out.println("Performance locating cubes : " + ((System.nanoTime() - tic) / Math.pow(10, 6)) + "ms");
				for (Cube cube : cubes)
					System.out.println("Cube found : " + cube);
				
			}
			
		}
				
	}
	
}