package autopilot_vision;

import java.awt.Color;
import java.util.ArrayList;

/**
 * A class of cube detection algorithms using more advanced logic.
 * 
 * 	The algorithms is written in C (called using JNI). It considers border points of any color and
 *   keeps the points with the (exact) same color in a respective list.
 *  Lists corresponding to the same cube (same hue and saturation) are the kept together in a struct 'Cube'.
 *  Each list is then processed using the Douglas-Peucker algorithm. The resulting polygon's edges
 *  	 are processed to filter out the 'true' edges (actual edges of a cube - those that do not get close to the frame
 *   or to an other Cube's edges).
 *  
 *  Upon return to Java, an actual cube list is returned and matched with the current hashmap of cubes.
 *   If an old cube was detected, its distance is updated, and, if necessary, its location is re-calculated
 *    if new information is available (at least one of its edges was detected).
 *   If a new cube is detected, its distance and location are set using either its bounding box or, if
 *    available, edge information.
 *   
 *  Border points of the same color are put in a separate list which is processed using the Douglas-Peucker
 *  	 algorithm.
 *  The resulting polygon's edges are processed. True edges of a cube are selected using basic logic.
 *  
 *  If no edges could be found, the bounding box of the cube is used to approximate distance to the cube.
 * 
 * @author	Team Saffier
 * @version 1.0
 * @note 	A Java version of the algorithm was made so that comparative speed-up can
 * 			be demonstrated. 
 */
public class CubeDetectionAlgorithmAdvanced extends CubeDetectionAlgorithm {
	
	/**
	 * Flag denoting whether or not the algorithm ignores colors that have already been seen in the past.
	 */
	private final static boolean USE_CACHE = true;
	
	/**
	 * Flag denoting whether or not the algorithm should use pure JAVA.
	 */
	private final static boolean USE_JAVA = false;
	
	@Override
	public ArrayList<Cube> locateUnitCubes(Image image) {
		
		if (USE_JAVA)
			return locateUnitCubesJava(image);
		
		// Pre-processing
		ArrayList<Cube> cubes = new ArrayList<Cube>();
		int width = image.getSize().getWidth(), height = image.getSize().getHeight();
	    	
		// Get 2D array with cube information from JNI
		// Array has [r,g,b - bounding box - edge coordinates] per row, one row for each cube
		int[][] cubesArray = CubeDetectionJNI.locateUnitCubes(image.getPixels(), width, height, USE_CACHE);
		float hsv[] = new float[3];
		Rectangle2D boundingBox;
		for (int i=0 ; i<cubesArray.length ; i++) {
						
			if (cubesArray[i].length < 7)
				continue;
			
			// Create new cube
			Color.RGBtoHSB(cubesArray[i][0], cubesArray[i][1], cubesArray[i][2], hsv);
			boundingBox = new Rectangle2D(cubesArray[i][3], cubesArray[i][5], cubesArray[i][4], cubesArray[i][6]);
			Cube cube = new Cube(boundingBox, hsv[0], hsv[1]);
			
			// Register edges that were found 
			for (int j=7 ; j+3 < cubesArray[i].length ; j+=4) {
				Line2D edge = new Line2D(new Point2D(cubesArray[i][j], cubesArray[i][j+1]),
										new Point2D(cubesArray[i][j+2], cubesArray[i][j+3]));
				// System.out.println(edge);
				cube.addEdge(edge);
			}
			
			// Add cube
			cubes.add(cube);
			
		}
				
		return cubes;
		
	}
	
	// For documentation see CubeDetectionAlgorithm.locateUnitCubes()
	// Algorithm forces the use of pure JAVA.
	private ArrayList<Cube> locateUnitCubesJava(Image image) {
		
		// Pre-processing
		ArrayList<Cube> cubes = new ArrayList<Cube>();
		int width = image.getSize().getWidth(), height = image.getSize().getHeight();
		
		byte[] pixels = image.getPixels();
		int len = pixels.length;
		int width3 = width*3;

		int i = 0, row = 0, column = 0;
	    byte r,g,b;
	    int borderpoints = 0;
	    while (i < len) {
	        
	    		r = pixels[i];
	    		g = pixels[i+1];
	    		b = pixels[i+2];
	    	
	    		if (r != -1 || g != -1 || b != -1) { // Not white
                // Check for border point
                if (column == 0 || column == width-1 || row == 0 || row == height-1
                    || r != pixels[i-3] || r != pixels[i+3] || r != pixels[i-width3] || r != pixels[i+width3]
                    || g != pixels[i-2] || g != pixels[i+4] || g != pixels[i-width3+1] || g != pixels[i+width3+1]
                    || b != pixels[i-1] || b != pixels[i+5] || b != pixels[i-width3+2] || b != pixels[i+width3+2]) {
                    borderpoints++;
                }
            }
	    		
	    		i += 3;
	    		
	        if (++column == width) {
	            column = 0;
	            row++;
	            // printf("new row");
	        }
	        
	    }
	    
	    System.out.println("Border points = " + borderpoints);

//	    BufferedImage bufferedImage = Image.getBufferedImageFromBytes(output, width, height);
//	    File file = new File("/Users/Bruno/Desktop/save"+(imageIndex++)+".bmp");
//		try {
//			ImageIO.write(bufferedImage, "bmp", file);
//		} catch (IOException e) {
//			e.printStackTrace();
//		}
		
		return cubes;
		
	}
		
	@Override
	public Cube locateUnitCube(Image image, float hue, float saturation) {
		
		// TODO : Unnecessary as assignment has gotten more complex than this
		return null;
		
	}
	
}