package autopilot_vision;

/**
 * A class of image sizes with a pixel width and - height.
 * 	The total amount of pixels corresponding to the image size cannot exceed Integer.MAX_VALUE.
 * 
 * @author	Team Saffier
 * @version	1.0
 *
 */
public class ImageSize {

	/**
	 * Initialise this new image size with given width and height.
	 * 
	 * @param 	width
	 * 			The image width in pixels.
	 * @param 	height
	 * 			The image height in pixels.
	 * @throws	IllegalArgumentException
	 * 			The given width is smaller than 1.
	 * 			| width < 1
	 * @throws	IllegalArgumentException
	 * 			The given height is smaller than 1.
	 * 			| height < 1
	 * @throws	IllegalArgumentException
	 * 			The number of pixels in this image size exceeds Integer.MAX_VALUE.
	 * 			| width != 0 && height > Integer.MAX_VALUE / width
	 */
	public ImageSize(int width, int height) {
		if (width < 1)
			throw new IllegalArgumentException("Image width is too small.");
		if (height < 1)
			throw new IllegalArgumentException("Image height is too small.");
		if (width != 0 && height > Integer.MAX_VALUE / width)
			throw new IllegalArgumentException("Image size is too large.");
		this.width = width;
		this.height = height;
	}
	
	/**
	 * Return the width of this image size.
	 */
	public int getWidth() {
		return this.width;
	}
	
	/**
	 * Variable registering the pixel width for this image size.
	 */
	private final int width;
	
	/**
	 * Return the height of this image size.
	 */
	public int getHeight() {
		return this.height;
	}
	
	/**
	 * Variable registering the pixel height for this image size.
	 */
	private final int height;
	
	/**
	 * Return the total number of pixels corresponding to this image size.
	 */
	public int getPixels() {
		return width * height;
	}
	
}