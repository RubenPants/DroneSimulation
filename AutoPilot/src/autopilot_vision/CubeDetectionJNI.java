package autopilot_vision;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import java.nio.file.Files;
import java.nio.file.StandardCopyOption;


/**
 * A class of cube detection algorithm JNI interfaces.
 *  Used by cube detection algorithms when wanting to detect cubes using C instead of JAVA.
 * 
 * @author	Team Saffier
 * @version 	1.0
 */
public class CubeDetectionJNI {

	/**
	 * Load the C library.
	 */
	static {
	    try {
	    	  	System.loadLibrary("autopilotVision");
	    } catch (UnsatisfiedLinkError e) {
	    	  	loadLibFromJar("autopilotVision");
	    }
	}
	
	/**
	 * Load the C library from JAR file.
	 * 
	 * @param name The name of the C library that is to be loaded.
	 */
	private static void loadLibFromJar(String name) {
		name = System.mapLibraryName(name);
	    try {
	    	
	    		System.out.println(name);
	    		InputStream in = CubeDetectionJNI.class.getResourceAsStream(name);
	    		File fileOut = new File(System.getProperty("java.io.tmpdir") + File.separator + name);
	    		Files.copy(in, fileOut.toPath(), StandardCopyOption.REPLACE_EXISTING);
	        in.close();
	        System.load(fileOut.toString());
	    	
//	    		URL urlIn = CubeDetectionJNI.class.getResource(name);
//	    		File fileIn = new File(urlIn.toString());
//	    		File fileOut = new File(System.getProperty("java.io.tmpdir") + File.separator + name);
//	    		IOUtils.copyFromURL(urlIn, fileOut);
//	    		Files.copy(fileIn.toPath(), fileOut.toPath(), StandardCopyOption.REPLACE_EXISTING);
//		    System.out.println("Writing dll to: " + fileOut.getAbsolutePath());
//	    		System.load(fileOut.toString());
	    		
	    } catch (Exception e) {
	        System.out.println("An error occured while reading the library : " + e.getLocalizedMessage());
	    }
	}
	
	/**
	 * Copy the given source file to the given destination file.
	 * 
	 * @param 	source
	 * 			The source file to copy.
	 * @param 	dest
	 * 			The destination file to copy to.
	 * @throws 	IOException
	 * 			If an I/O error occurred.
	 */
	private static void copyFileUsingFileStreams(File source, File dest) throws IOException {
		InputStream input = null;
		OutputStream output = null;
		try {
			input = new FileInputStream(source);
			output = new FileOutputStream(dest);
			byte[] buf = new byte[1024];
			int bytesRead;
			while ((bytesRead = input.read(buf)) > 0) {
				output.write(buf, 0, bytesRead);
			}
		} catch (Exception e) {
			System.out.println("Error copying : " + e.getLocalizedMessage());
		} finally {
			input.close();
			output.close();
		}
	}
	
	/**
	 * Set up JNI.
	 */
	public static native void setup();
	
	/**
	 * Clean up JNI when it is no longer needed.
	 */
	public static native void cleanup();
	
	/**
	 * Locate all unit cubes in the given image with given parameters.
	 * 	If history is used, then cubes whose edges were previously seen are ignored.
	 * 
	 * @param 	pixels
	 * 			The image's pixels.
	 * @param 	width
	 * 			The image width.
	 * @param 	height
	 * 			The image height.
	 * @param 	useCache
	 * 			Flag denoting whether or not previously seen cubes are ignored if enough information
	 * 			about them was gathered.
	 */
	public static native int[][] locateUnitCubes(byte[] pixels, int width, int height, boolean useCache);
	
	/**
	 * Convenience class matching a C struct used by the JNI algorithm.
	 * 
	 * @author 	Team Saffier
	 * @version	1.0
	 */
//	private class CubeJNI {
//		@SuppressWarnings("unused") public float[] color;
//		@SuppressWarnings("unused") public int[] boundingBox;
//		@SuppressWarnings("unused") public float[] edges;
//	}
	
}