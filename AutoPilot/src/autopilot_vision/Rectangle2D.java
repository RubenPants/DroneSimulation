package autopilot_vision;

import java.lang.IllegalArgumentException;
import java.util.ArrayList; 

/**
 * A class representing a rectangle in 2-dimensional Euclidian space, 
 * 	having a lower left - and an upper right vertex.
 * 
 * @author Team Saffier
 * @version 1.0
 */
public class Rectangle2D {

	/**
	 * Initializes a new rectangle with given lower left and upper right vertices.
	 * 
	 * @param 	minX
	 *        	The x coordinate of the rectangle's lower left vertex.
	 * @param 	maxX
	 *        	The x coordinate of the rectangle's upper right vertex.
	 * @param 	minY
	 *        	The y coordinate of the rectangle's lower left vertex.
	 * @param 	maxY
	 *        	The y coordinate of the rectangle's upper right vertex.
	 * @post   	The x coordinate of this rectangle's lower left vertex equals the given one.
	 *       	| new.getMinimumX() == minX
	 * @post   	The x coordinate of this rectangle's upper right vertex equals the given one.
	 *       	| new.getMaximumX() == maxX
	 * @post   	The y coordinate of this rectangle's lower left vertex equals the given one.
	 *       	| new.getMinimumY() == minY
	 * @post   	The y coordinate of this rectangle's upper right vertex equals the given one.
	 *       	| new.getMaximumY() == maxY
	 * @throws	IllegalArgumentException
	 * 			The given lower left vertex does not lie below and on the left of the given upper right vertex.
	 */
	public Rectangle2D(int minX, int maxX, int minY, int maxY) throws IllegalArgumentException {
		if (minX > maxX || minY > maxY)
			throw new IllegalArgumentException("Invalid rectangle coordinates : (" + minX + "," + maxX + ","+ minY + ","+ maxY + ")");
		this.minX = minX;
		this.maxX = maxX;
		this.minY = minY;
		this.maxY = maxY;
	}

	/**
	 * Returns the x coordinate of the lower left vertex of this rectangle.
	 */
	public int getMinimumX() {
		return this.minX;
	}
	
	/**
	 * A variable registering the x coordinate of the lower left vertex of this rectangle.
	 */
	private int minX;
	
	/**
	 * Returns the x coordinate of the upper right vertex of this rectangle.
	 */
	public int getMaximumX() {
		return this.maxX;
	}
	
	/**
	 * A variable registering the x coordinate of the upper right vertex of this rectangle.
	 */
	private int maxX;
	
	/**
	 * Returns the y coordinate of the lower left vertex of this rectangle.
	 */
	public int getMinimumY() {
		return this.minY;
	}
	
	/**
	 * A variable registering the y coordinate of the lower left vertex of this rectangle.
	 */
	private int minY;
	
	/**
	 * Returns the y coordinate of the upper right vertex of this rectangle.
	 */
	public int getMaximumY() {
		return this.maxY;
	}
	
	/**
	 * A variable registering the y coordinate of the upper right vertex of this rectangle.
	 */
	private int maxY;
	
	/**
	 * Returns the center of this rectangle.
	 * 
	 * @return	A point representing the center of this rectangle.
	 * 			| new Point2D(minX + (maxX - minX), minY + (maxY - minY))
	 */
	public Point2D getCenter() {
		return new Point2D(minX + (maxX - minX) / 2, minY + (maxY - minY) / 2);
	}
	
	/**
	 * Returns the top left corner of this rectangle.
	 * 
	 * @return	A new 2-dimensional point with the top left coordinates of this rectangle.
	 * 			| new Point2D(minX, maxY);
	 */
	public Point2D getTopLeftCorner() {
		return new Point2D(minX, maxY);
	}
	
	/**
	 * Returns the top left corner of this rectangle.
	 * 
	 * @return	A new 2-dimensional point with the top right coordinates of this rectangle.
	 * 			| new Point2D(maxX, maxY);
	 */
	public Point2D getTopRightCorner() {
		return new Point2D(maxX, maxY);
	}
	
	/**
	 * Returns the bottom left corner of this rectangle.
	 * 
	 * @return	A new 2-dimensional point with the bottom left coordinates of this rectangle.
	 * 			| new Point2D(minX, minY);
	 */
	public Point2D getBottomLeftCorner() {
		return new Point2D(minX, minY);
	}
	
	/**
	 * Returns the bottom right corner of this rectangle.
	 * 
	 * @return	A new 2-dimensional point with the bottom right coordinates of this rectangle.
	 * 			| new Point2D(maxX, minY);
	 */
	public Point2D getBottomRightCorner() {
		return new Point2D(maxX, minY);
	}
	
	/**
	 * Get the edges of this cube as 2-dimensional lines.
	 * 
	 * @return	A list with the edges of this rectangle.
	 */
	public ArrayList<Line2D> getEdges() {
		ArrayList<Line2D> edges = new ArrayList<Line2D>();
		edges.add(new Line2D(getTopLeftCorner(), getTopRightCorner()));
		edges.add(new Line2D(getTopRightCorner(), getBottomRightCorner()));
		edges.add(new Line2D(getBottomRightCorner(), getBottomLeftCorner()));
		edges.add(new Line2D(getBottomLeftCorner(), getTopLeftCorner()));
		return edges;
	}
	
	/**
	 * Alter this rectangle to incorporate the given 2-dimensional point.
	 */
	public void spanPoint(int x, int y) {
		if (x < minX) minX = x;
		else if (x > maxX) maxX = x;
		if (y < minY) minY = y;
		else if (y > maxY) maxY = y;
	}
	
	/**
	 * Checks whether the given rectangle overlaps with this rectangle.
	 * 
	 * @param  	other
	 *         	The rectangle to check with.
	 * @return 	True if and only if the given rectangle overlaps this rectangle.
	 *       	| result == (other.maxX >= this.minX
	 *			&& other.minX <= this.maxX
	 *			&& other.maxY >= this.minY
	 *			&& other.minY <= this.maxY)
	 * @note	This method does *not* check for intersection, as one rectangle inside an other
	 * 			 is not an intersection while it is an overlap.
	 */
	public boolean overlaps(Rectangle2D other) { // Check if any of the minima/maxima conflict, 4 comparisons.
		return (other.maxX >= this.minX
				&& other.minX <= this.maxX
				&& other.maxY >= this.minY
				&& other.minY <= this.maxY);
	}
	
	/**
	 * Returns a string representation of this rectangle.
	 * 
	 * @return 	A string representing this rectangle.
	 * 			| result == "[" + this.getMinimumX() + " " + this.getMaximumX() + " " + this.getMinimumY() + " " + this.getMaximumY() + "]"
	 */
	public String toString() {
		return "[" + this.getMinimumX() + " " + this.getMaximumX() + " " + this.getMinimumY() + " " + this.getMaximumY() + "]";
	}

}