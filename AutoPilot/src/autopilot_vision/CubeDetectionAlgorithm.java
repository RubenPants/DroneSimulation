package autopilot_vision;

import java.util.ArrayList;

/**
 * An abstract class of algorithms for locating 3-dimensional cubes in a 2-dimensional image.
 * 
 * @author	Team Saffier
 * @version 	1.0
 */
public abstract class CubeDetectionAlgorithm {
	
	/**
	 * An enumeration with types of algorithms implemented throughout.
	 */
	public static enum CubeDetectionAlgorithmType { // Needs to be static to be able to access it in other classes.
		
		BASIC,				// Basic detection algorithm
		ADVANCED;			// Basic detection algorithm
		
		/**
		 * Returns the type of algorithm matching the given ordinal.
		 */
		public static CubeDetectionAlgorithmType typeForOrdinal(int ordinal) {
			if (ordinal < 0 || ordinal > 1)
				return null;
			else
				return CubeDetectionAlgorithmType.values()[ordinal];
		}
		
	}
	
	/**
	 * Create and initialize a new algorithm of given type.
	 * 
	 * @param 	type
	 * 			The type of cube recognition algorithm to set up.
	 * @return	An initialized algorithm of given type, or null if the given type is not valid.
	 */
	public static CubeDetectionAlgorithm initializeAlgorithm(CubeDetectionAlgorithmType type) {

		if (type == null)
			return null; // null type given
		
		switch (type) {
			case BASIC:
				return new CubeDetectionAlgorithmBasic();
			case ADVANCED:
				return new CubeDetectionAlgorithmAdvanced();
			default:
				return null; // No valid type given
		}
		
	}
	
	/**
	 * Locate a cube with given hue and saturation in the given image.
	 * 
	 * @param 	image
	 * 			The image in which the unit cube is to be located.
	 * @param 	hue
	 * 			The hue of the color of the unit cube that is to be located.
	 * @param 	saturation
	 * 			The saturation of the color of the unit cube that is to be located.
	 * @return	A cube with given hue/saturation, or null if no such cube was detected.
	 */
	public abstract Cube locateUnitCube(Image image, float hue, float saturation);

	/**
	 * Threshold when comparing hue and saturation of cube sides.
	 */
	public final static float THRESHOLD = 0.02f;
	
	/**
	 * Locate unit cubes in the given image.
	 * 
	 * @param 	image
	 * 			The image in which the unit cube is to be located.
	 * @return	A collection of cubes located in the given image.
	 */
	public abstract ArrayList<Cube> locateUnitCubes(Image image);
	
}