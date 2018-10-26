package autopilot_vision;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

/**
 * A class of images having an array of bytes having the color of its pixels (RGB format),
 * 	as well as a variety of parameters of the camera with which it was taken.
 * 
 * @author	Team Saffier
 * @version	1.0
 *
 */
public class Image {

	/**
	 * Initialise this new image with given byte array and given image size.
	 * 
	 * @param 	pixels
	 * 			The byte array having this image pixels' RGB values.
	 * @param 	size	
	 * 			The size of this new image.
	 * @return	A new image with given RGB values and given size.
	 * @throws	IllegalArgumentException
	 * 			The given byte array's length does not correspond to the given image size.
	 * 			| size.getPixels() * 3 != pixels.length
	 */
	public Image (byte[] pixels, ImageSize size) {
		if (size.getPixels() * 3 != pixels.length)
			throw new IllegalArgumentException("Pixel count does not correspond to the given image size.");
		this.pixels = pixels;
		this.size = size;
	}
	
	/**
	 * Return the byte array with the RGB values of this image's pixels (left to right, top to bottom).
	 */
	public byte[] getPixels() {
		return this.pixels;
	}
	
	/**
	 * The byte array with the RGB values of this image's pixels (left to right, top to bottom).
	 */
	private byte[] pixels;
	
	/**
	 * Get the size of this image.
	 */
	public ImageSize getSize() {
		return this.size;
	}
	
	/**
	 * Variable registering the size of this image.
	 */
	private ImageSize size;

	/**
	 * Create an image with the contents of the given file.
	 * 
	 * @param 	file
	 * 			The file that is to be read from.
	 * @return	A new image object corresponding to the file at given path,
	 * 			 or null if an I/O error occurred.
	 * @category	Class
	 */
	public static Image createImageUsingFile(File file) {

		BufferedImage image;
		try {
			image = ImageIO.read(file);
			return Image.getImageFromBufferedImage(image);
		} catch (IOException exception) {
			System.out.println("An I/O error occured while reading the file at '" + file.getPath() + "'");
		}

		return null;

	}

	/**
	 * Convert a buffered image to an image.
	 * 
	 * @param 	image
	 * 			The image to convert.
	 * @return	An image with an array of bytes having the RGB values for the pixels of the given image,
	 * 			 left to right and top to bottom.
	 * @note		This method assumes that the given image is not a monochrome bitmap image.
	 * @category	Class
	 */
	private static Image getImageFromBufferedImage(BufferedImage image) {		

		// Pre-processing
		final int width = image.getWidth(), height = image.getHeight();
		byte[] newPixels = new byte[width*height*3];

		/*
			int pixel = 0, rgb;
			for (int row=0 ; row<height ; row++) {
				for (int column=0 ; column<width ; column++) {
					rgb = image.getRGB(column, row);
					newPixels[pixel] = (byte)((rgb & 0xff0000) >> 16);
					newPixels[pixel + 1] = (byte)((rgb & 0xff00) >> 8);
					newPixels[pixel + 2] = (byte)(rgb & 0xff);
					pixel += 3;
				}
			}
		 */

		final byte[] pixels = ((DataBufferByte)image.getRaster().getDataBuffer()).getData();
		final boolean hasAlphaChannel = image.getAlphaRaster() != null;

		// Calculate byte array
		int pixel = (hasAlphaChannel ? 1 : 0), pixelStep = (hasAlphaChannel ? 4 : 3);
		for (int newPixel = 0; newPixel<width*height*3 ; newPixel += 3) {
			newPixels[newPixel] = pixels[pixel + 2]; // Red
			newPixels[newPixel + 1] = pixels[pixel + 1]; // Green
			newPixels[newPixel + 2] = pixels[pixel]; // Blue
			pixel += pixelStep;
		}

		return new Image(newPixels, new ImageSize(width, height));

	}
	
	/**
	 * Convert a byte array with rgb values to an image.
	 * 
	 * @param 	image
	 * 			The byte array with rgb values.
	 * @param	width
	 * 			The width of the image.
	 * @param	height
	 * 			The height of the image.
	 * @return	A buffered image with corresponding to the given byte array.
	 * @category	Class
	 */
	public static BufferedImage getBufferedImageFromBytes(byte[] image, int width, int height) {		

		// Convert to image
		BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
		for (int i=0 ; i<height ; i++) {
			for (int j=0 ; j<width ; j++) {
				byte r = image[(i * width + j) * 3 + 0];
				byte g = image[(i * width + j) * 3 + 1];
				byte b = image[(i * width + j) * 3 + 2];
				int rgb = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
				bufferedImage.setRGB(j, height - 1 - i, rgb);
			}
			// System.out.println();
		}

		return bufferedImage;
		
	}

	/**
	 * Converts an array of bytes representing RGB values to an array of floats
	 *  having the corresponding HSV values.
	 * 
	 * @param 	pixels
	 * 			The byte array representing the RGB values of an image.
	 * @return	An array of floats of the same length as the input array, containing
	 * 			 the corresponding HSV values.
	 */
	public static float[] convertRGBToHSV(byte[] pixels){

		// Pre-processing
		int i = 0, length = pixels.length;
		float[] output = new float[length];
		float hue, saturation, brightness;

		// Convert pixel data
		while (i < length) {

			// Convert RGB to HSV
			int red = (pixels[i] & 0xFF), green = (pixels[i+1] & 0xFF), blue = (pixels[i+2] & 0xFF);
			int cmax = (red > green) ? red : green;
			if (blue > cmax) cmax = blue;
			int cmin = (red < green) ? red : green;
			if (blue < cmin) cmin = blue;
			brightness = ((float) cmax) / 255.0f;
			if (cmax != 0) saturation = ((float) (cmax - cmin)) / ((float) cmax);
			else saturation = 0;
			if (saturation == 0) hue = 0;
			else {
				float redc = ((float) (cmax - red)) / ((float) (cmax - cmin));
				float greenc = ((float) (cmax - green)) / ((float) (cmax - cmin));
				float bluec = ((float) (cmax - blue)) / ((float) (cmax - cmin));
				if (red == cmax) hue = bluec - greenc;
				else if (green == cmax) hue = 2.0f + redc - bluec;
				else hue = 4.0f + greenc - redc;
				hue = hue / 6.0f;
				if (hue < 0) hue = hue + 1.0f;
			}

			// Add HSV values
			output[i] = hue;
			output[i+1] = saturation;
			output[i+2] = brightness;	
			i += 3;

		}

		return output;

	}
	
}