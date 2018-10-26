package autopilot_vision;

import java.util.ArrayList;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;
import org.lwjgl.util.vector.Vector4f;

import interfaces.AutopilotConfig;
import interfaces.AutopilotInputs;
import autopilot_vision.CubeDetectionAlgorithm.CubeDetectionAlgorithmType;

/**
 * A class for autopilot camera image analysers, recognizing objects and locating them.
 * 
 * @author 	Team Saffier
 * @version 	1.0
 */
public class AutopilotImageAnalyser {
	
	/**
	 * Initialise this new image analyser with a basic cube recognition algorithm, a
	 * 	default width & height for input images, and a default horizontal & vertical angle
	 * 	of view of these very images.
	 * 
	 * @effect	This image analyser is initialised with a basic cube recognition algorithm.
	 */
	public AutopilotImageAnalyser() {
		this(CubeDetectionAlgorithmType.ADVANCED);
	}
	
	/**
	 * Initialise this new image analyser with given input image size, input image
	 *  horizontal/vertical angle of view and cube recognition algorithm type.
	 *  
	 * @param 	type
	 * 			The type of cube detection algorithm that is to be used by this new
	 * 			 image analyser.
	 */
	public AutopilotImageAnalyser(CubeDetectionAlgorithmType type) {
		this.cubeDetectionAlgorithm = CubeDetectionAlgorithm.initializeAlgorithm(type);
	}
	
	/**
	 * Variable registering an algorithm used by this image analyser for detecting and locating cubes.
	 */
	private CubeDetectionAlgorithm cubeDetectionAlgorithm;
		
	/**
	 * Locate all the cubes in an input image.
	 * 
	 * @param 	configuration
	 * 			The configuration of the autopilot.
	 * @param 	input
	 * 			The latest input handed to the autopilot.
	 * @param	useHistory
	 * 			Whether or not previously seen cubes (but not visible) are to be considered.
	 * @return	An array with 2 values - requested pitch and heading -
	 * 			representing the next move for the autopilot.
	 */
	public ArrayList<Cube> locateAllCubes(AutopilotConfig configuration, AutopilotInputs input, boolean useHistory) {
	
		// Detect cube
		ImageSize inputSize = new ImageSize(configuration.getNbColumns(), configuration.getNbRows());
		Image inputImage = new Image(input.getImage(), inputSize);
		ArrayList<Cube> cubes = cubeDetectionAlgorithm.locateUnitCubes(inputImage);
				
		// Calculate distances/positions
		Vector3f droneCoordinates = new Vector3f(input.getX(), input.getY(), input.getZ());
		Matrix4f transformationMatrix = getDroneToWorldTransformationMatrix(
				input.getHeading(), 
				input.getPitch(), 
				input.getRoll());
		for (Cube cube : cubes) {
			cube.setVisible(true);
			if (updateDistanceToCube(cube, 
					configuration.getHorizontalAngleOfView(),
					configuration.getNbColumns(),
					configuration.getNbRows()))
				cube.setLocation(approximateLocation(cube,
						cube.getDistance(),
						droneCoordinates,
						transformationMatrix,
						configuration.getNbColumns(),
						configuration.getNbRows(),
						configuration.getHorizontalAngleOfView(),
						configuration.getVerticalAngleOfView()));
			else {
				cube.setDistance(-1.0);
				// cube.setLocation(null);
			}
		}

		// Reset history (make all previously seen cubes invisible)
		// 	and update it with the cubes that have just been seen
		updateCubeList(cubes);
		
		// Removal of cubes (should be in updateCubeList, combine logic)
		// If a cube is invisible but considered really close-by, then it is removed
		// If it is seen again it will be re-added / if it truly disappeared, great
		Point3D currentLocation = new Point3D(input.getX(), input.getY(), input.getZ());
		ArrayList<Cube> toDelete = new ArrayList<Cube>();
		for (Cube cube : history) { 
			if (!cube.isVisible()
				&& cube.getLocation() != null 
				&& cube.getLocation().distanceTo(currentLocation) < 2)
				toDelete.add(cube);
		}
		history.removeAll(toDelete);
		
		return (useHistory ? history : cubes);
		
	}
	
	/**
	 * Remove all history from this analyser (i.e. all previously seen cubes).
	 */
	public void clearHistory() {
		history.clear();
	}
	
	/**
	 * Update the current list of cubes using the given list of cubes that have just been detected..
	 * 
	 * @param 	cubes
	 * 			The list of cubes that was just detected.
	 */
	protected void updateCubeList(ArrayList<Cube> cubes) {
		
		// Make old cubes invisible
		for (Cube cube : history)
			cube.setVisible(false);
		
		// Tag the ones recently detected as visible
		// New cubes are simply added to the list
		boolean added;
		for (Cube cube : cubes) {
			added = false;
			for (Cube oldCube : history) {
				float huediff = Math.abs(cube.getHue() - oldCube.getHue());
				float satdiff = Math.abs(cube.getSaturation() - oldCube.getSaturation());
				if ((huediff < CubeDetectionAlgorithm.THRESHOLD || 1.0 - huediff < CubeDetectionAlgorithm.THRESHOLD)
					&& (satdiff < CubeDetectionAlgorithm.THRESHOLD || 1.0 - satdiff < CubeDetectionAlgorithm.THRESHOLD)) {
					oldCube.setVisible(true);
					oldCube.setBoundingBox(cube.getBoundingBox());
					if (cube.getLocation() != null) {
						oldCube.setDistance(cube.getDistance());
						oldCube.setLocation(cube.getLocation());
					}
					added = true;
					break;
				}
			}
			if (!added) { // New  cube, add it to the list
				cube.setVisible(true);
				history.add(cube);
			}
		}
		
	}
	
	/**
	 * A list of cubes that have been seen so far.
	 * 
	 * @note 	This could be implemented using other data structures like hashmaps but 
	 * 			this is not necessary for now ; it'll matter if 100 or more cubes are regularly seen
	 * 			on an image.
	 */
	protected ArrayList<Cube> history = new ArrayList<Cube>();
		
	/**
	 * Locate a red unit cube in the image in the given input, keeping in mind the given configuration
	 * 	of the autopilot.
	 * 
	 * @param 	configuration
	 * 			The autopilot's configuration.
	 * @param 	input
	 * 			The autopilot's input that is to be analysed.
	 * @return	An integer array with the center coordinates of the red cube, as well as its
	 * 				approximate distance to the camera, or null if no cube was found.
	 * 				[0]x [1]y [2]vertical angle [3]horizontal angle [4]distance

	 */
	public Cube locateRedCube(AutopilotConfig configuration, AutopilotInputs input) {
		
		// Read input
		ImageSize inputSize = new ImageSize(configuration.getNbColumns(), configuration.getNbRows());
		Image inputImage = new Image(input.getImage(), inputSize);
		return cubeDetectionAlgorithm.locateUnitCube(inputImage, 0.0f, 1.0f);

	}
	
	/**
	 * Approximate the distance of the given cube to the drone with given configuration.
	 *  This method applies basic logic to see if it is worthwhile to update the cube's distance.
	 *  Worthwhile meaning that enough and better information has been gathered.
	 * 
	 * @param 	cube
	 * 			The cube whose distance is to be approximated.
	 * @param 	haov
	 * 			The horizontal angle of view of the drone.
	 * @param 	cols
	 * 			The horizontal resolution of the drone.
	 * @param 	rows
	 * 			The vertical resolution of the drone.
	 * @return	True if and only if the distance to the cube was updated
	 */
	public static boolean updateDistanceToCube(Cube cube, float haov, int cols, int rows) {
		
		// Calculate camera's distance to projection plane (l)
		double alpha = Math.PI / 2 - haov;
		double l = Math.tan(alpha) * (cols / 2);
		
		// If edges are available, the longest one is selected
		Line2D validEdge = null;
		int edgeCount = cube.getNbEdges();
		if (edgeCount > 0) {
			Line2D edge = cube.getEdgeAtIndex(0);
			for (int i=1 ; i<edgeCount ; i++) {
				if (edge.length() < cube.getEdgeAtIndex(i).length() && isValidEdge(edge, rows, cols))
					edge = cube.getEdgeAtIndex(i);
			}
			if (isValidEdge(edge, rows, cols))
				validEdge = edge;
		}
		
		// If no 'real' edges can be used, use the bounding box
		boolean boundingBox = false;
		if (validEdge == null) {
			for (Line2D edge : cube.getBoundingBox().getEdges())
				if (isValidEdge(edge, rows, cols))
					validEdge = edge;
			if (validEdge != null)
				boundingBox = true;
			else
				return false;
		}
				
		// A valid edge has been found, get its starting points and use them for distance calculation
		Point2D firstPoint = validEdge.getStartPoint(), lastPoint = validEdge.getEndPoint();
		Point2D imageCenter = new Point2D(cols / 2, rows / 2);
		
		// Determine lengths of projection arms from camera to edge
		double distFirstCenter = firstPoint.distanceTo(imageCenter);
		double distLastCenter = lastPoint.distanceTo(imageCenter);
		double distFirstLast = firstPoint.distanceTo(lastPoint);
		double armlengthFirst = Math.sqrt(l*l + distFirstCenter*distFirstCenter);
		double armlengthLast = Math.sqrt(l*l + distLastCenter*distLastCenter);
		
		// Determine angle under which edge is projected (law of cosine)
		double gamma = Math.acos((distFirstLast*distFirstLast
				- armlengthFirst*armlengthFirst
				- armlengthLast*armlengthLast) / ((-2) * armlengthFirst * armlengthLast));
				
		// Determine approximate distance (law of sine)
		alpha = (Math.PI - gamma) / 2;
		double distance = Math.sin(alpha) / Math.sin(gamma);	
		
		// If distance was far away, normalise it
		if (Double.isInfinite(distance) || Double.isNaN(distance))
			distance = 100;
		
		// If cube has no location registered or currently measured distance is closer, update
		if (cube.getLocation() == null || distance < cube.getDistance()) {
			cube.setDistance(distance);
			cube.distanceUsesBoundingBox = boundingBox;
			return true;
		}
		
		return false;
		
	}
	
	/**
	 * Check whether the given edge can be used for distance approximation.
	 *  This is true if and only if it does not touch the border of the frame.
	 *  
	 * @param 	edge
	 * 			The edge to check for.
	 * @param 	rows
	 * 			The vertical resolution of the frame.
	 * @param 	cols
	 * 			The horizontal resolution of the frame.
	 */
	private static boolean isValidEdge(Line2D edge, int rows, int cols) {
		Point2D firstPoint = edge.getStartPoint(), lastPoint = edge.getEndPoint();
		if (firstPoint.getX() > 0 && firstPoint.getY() > 0 
			&& firstPoint.getX() < cols-1 && firstPoint.getY() < rows-1
			&& lastPoint.getX() > 0 && lastPoint.getY() > 0 
			&& lastPoint.getX() < cols-1 && lastPoint.getY() < rows-1) {
			return true;
		}
		return false;
	}
	
	/**
	 * Approximate the 3-dimensional location of the given cube corresponding to its given distance from a camera
	 *  with given current situation.
	 *  
	 *  @param	cube
	 *  			The cube whose distance is to be determined.
	 *  @param	distance
	 *  			The distance of the cube to the camera.
	 *  @param	cameraCoordinates
	 *  			The coordinates of the camera in the world.
	 *  @param	cameraToWorldTransformationMatrix
	 *  			The matrix to transform from camera to world coordinates.
	 *  @param 	cols
	 *  			The horizontal resolution of the camera.
	 *  @param 	rows
	 *  			The vertical resolution of the camera.
	 *  @param 	haov
	 *  			The horizontal angle of view of the camera.
	 *  @param 	vaov
	 *  			The vertical resolution of the camera.
	 */
	public static Point3D approximateLocation(Cube cube, 
			double distance,
			Vector3f cameraCoordinates,
			Matrix4f cameraToWorldTransformationMatrix, 
			int cols,
			int rows,
			float haov,
			float vaov) {
		
		// Calculate pitch and heading of cube relative to the drone
		Point2D center = cube.getCenter();
		double horizontal = (center.getX() - cols/2)/(cols/2), vertical = (rows/2 - center.getY())/(rows/2);
		float pitchForCube = (float)(vertical * vaov / 2);
		float headingForCube = - (float)(horizontal * haov / 2);
		
		// Calculate vector directed from the drone towards the cube
		Vector3f cubeVector = new Vector3f(
					(float)-Math.sin(headingForCube),
					(float)Math.sin(pitchForCube),
					(float)-Math.cos(headingForCube)
				);
		cubeVector.normalise(cubeVector);
		// System.out.println("NORM " + pitchForCube + " --- " + headingForCube + " --- " + cubeVector);
		cubeVector.scale((float)distance);
		// System.out.println("SCALED " + pitchForCube + " --- " + headingForCube + " --- " + cubeVector);
				
		// Transform cube vector from drone to world
		Vector4f cubeVectorTransform = new Vector4f(
				cubeVector.x, 
				cubeVector.y, 
				cubeVector.z, 
				1);
		cubeVectorTransform = Matrix4f.transform(cameraToWorldTransformationMatrix, cubeVectorTransform, null);
//		System.out.println("TRANSFORM " + pitchForCube + " --- " + headingForCube + " --- " + cubeVectorTransform);
		
		return new Point3D(
				cameraCoordinates.getX() + cubeVectorTransform.x,
				cameraCoordinates.getY() + cubeVectorTransform.y,
				cameraCoordinates.getZ() + cubeVectorTransform.z
				);
		
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
		Matrix4f.rotate((float)heading, new Vector3f(0, 1, 0), matrix, matrix);
		Matrix4f.rotate((float)pitch, new Vector3f(1, 0, 0), matrix, matrix);
		Matrix4f.rotate((float)roll, new Vector3f(0, 0, 1), matrix, matrix);
		// Matrix4f.invert(matrix, null);
		return matrix;
	}
	
}