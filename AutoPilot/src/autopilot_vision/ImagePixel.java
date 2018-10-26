package autopilot_vision;

/**
 * A class representing a pixel in 2-dimensional image, having an x and an y coordinate.
 * 
 * @author Team Saffier
 * @version 1.0
 */
public class ImagePixel {

	/**
	 * Initialize a new pixel with given x and given y coordinates.
	 * 
	 * @param  	x
	 *		   	The x coordinate for this new point.
	 * @param  	y
	 *		   	The y coordinate for this new point.
	 * @post   	The x coordinate of this new point is equal to
	 *		   	the given x coordinate.
	 *       	| new.getX() == x
	 * @post   	The y coordinate of this new point is equal to
	 *		   	the given y coordinate.
	 *       	| new.getY() == y
	 */
	public ImagePixel(int x, int y) {
		this.x = x;
		this.y = y;
	}
	
	/**
	 * Return the x coordinate of this pixel.
	 */
	public int getX() {
		return x;
	}
	
	/**
	 * Variable registering the x coordinate of this pixel.
	 */
	private int x;

	/**
	 * Return the y coordinate of this pixel.
	 */
	public int getY() {
		return y;
	}

	/**
	 * Variable registering the y coordinate of this pixel.
	 */
	private int y;
	
	/**
	 * Calculates the Euclidian distance between this pixel and the given one.
	 * 
	 * @param 	other
	 * 			The pixel to calculate the distance from.
	 * @return	The Euclidian distance between this - and the given pixel.
	 * @note		The distance is not calculated using the classical sqrt(dx^2+dy^2) equation as to prevent overflow.
	 */
	public double distanceTo(ImagePixel other) {
		double dx = Math.abs(this.getX() - other.getX()), dy = Math.abs(this.getY() - other.getY());
		double min = Math.min(dx, dy), max = Math.max(dx, dy);
		if (max == 0)
			return 0;
		double r = min / max;
		return max * Math.sqrt(1 + r*r);
	}
	
	/**
	 * Returns a string representation of this pixel.
	 * 
	 * @return 	A string representing this pixel.
	 * 			| result == "[" + this.getX() + " " + this.getY() + "]"
	 */
	public String toString() {
		return "[" + this.getX() + " " + this.getY() + "]";
	}
	
	/**
	 * Check if two pixels are equal.
	 * 
	 * @param 	other
	 * 			The other pixel that is compared with this pixel.
	 * @return	Whether both x- and y-coordinates are equal.
	 * 			| this.getX() == other.getX() && this.getY() == other.getY()
	 */
	public boolean equalsTo(ImagePixel other) {
		return (this.getX() == other.getX() && this.getY() == other.getY());
	}

}