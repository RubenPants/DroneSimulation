package autopilot_vision;

/**
 * A class representing a point in 3-dimensional Euclidian space, having an x, an y and a z coordinate.
 * 
 * @author Team Saffier
 * @version 1.0
 */
public class Point3D {

	/**
	 * Initialize a new point with given x and given y coordinates.
	 * 
	 * @param  	x
	 *		   	The x coordinate for this new point.
	 * @param  	y
	 *		   	The y coordinate for this new point.
	 * @param  	z
	 *		   	The z coordinate for this new point.
	 * @post   	The x coordinate of this new point is equal to
	 *		   	the given x coordinate.
	 *       	| new.getX() == x
	 * @post   	The y coordinate of this new point is equal to
	 *		   	the given y coordinate.
	 *       	| new.getY() == y
	 * @post   	The z coordinate of this new point is equal to
	 *		   	the given z coordinate.
	 *       	| new.getZ() == z
	 */
	public Point3D(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	/**
	 * Return the x coordinate of this point.
	 */
	public double getX() {
		return x;
	}
	
	/**
	 * Variable registering the x coordinate of this point.
	 */
	private double x;

	/**
	 * Return the y coordinate of this point.
	 */
	public double getY() {
		return y;
	}

	/**
	 * Variable registering the y coordinate of this point.
	 */
	private double y;
	
	/**
	 * Return the z coordinate of this point.
	 */
	public double getZ() {
		return z;
	}

	/**
	 * Variable registering the z coordinate of this point.
	 */
	private double z;
	
	/**
	 * Calculates the Euclidian distance between this point and the given one.
	 * 
	 * @param 	other
	 * 			The point to calculate the distance from.
	 * @return	The Euclidian distance between this - and the given point.
	 */
	//public double distanceTo(Point3D other) {
	//	double dx = Math.abs(this.getX() - other.getX()), dy = Math.abs(this.getY() - other.getY());
	//	double min = Math.min(dx, dy), max = Math.max(dx, dy);
	//	if (max == 0)
	//		return 0;
	//	double r = min / max;
	//	return max * Math.sqrt(1 + r*r);
	//}
	
	/**
	 * Calculate the distance between this point and the other given point.
	 * 
	 * @param 	other
	 * 			The given point to calculate the distance to.
	 * @return	The distance between this point end the given one.
	 */
	public double distanceTo(Point3D other) {
		return Math.sqrt(Math.pow(this.getX() - other.getX(), 2)
				+ Math.pow(this.getY() - other.getY(), 2)
				+ Math.pow(this.getZ() - other.getZ(), 2));
	}
	
	/**
	 * Returns a string representation of this point.
	 * 
	 * @return 	A string representing this point.
	 * 			| result == "[" + this.getX() + " " + this.getY() + " " + this.getZ() + "]";
	 */
	public String toString() {
		return "[" + this.getX() + " " + this.getY() + " " + this.getZ() + "]";
	}
	
	/**
	 * Check if two points are equal.
	 * 
	 * @param 	other
	 * 			The other point that is compared with this point.
	 * @return	Whether the x-, y- and z-coordinates are equal.
	 * 			| this.getX() == other.getX() && this.getY() == other.getY() && this.getZ() == other.getZ()
	 */
	public boolean equalsTo(Point3D other) {
		return (this.getX() == other.getX() && this.getY() == other.getY() && this.getZ() == other.getZ());
	}

}