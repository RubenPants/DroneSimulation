package autopilot_vision;

/**
 * A class representing a line in 2-dimensional Euclidian space, having a 'begin' and 'end' point.
 * 
 * @author Team Saffier
 * @version 1.0
 */
public class Line2D {

	/**
	 * Initialize a new line with given 'begin' and 'end' points.
	 * 
	 * @param  	startPoint
	 *		   	The start point for this new line.
	 * @param  	endPoint
	 *		   	The end point for this new line.
	 * @post   	The start point for this new line equals the
	 * 			given start point.
	 *       	| new.getStartPoint() == startPoint
	 * @post   	The end point for this new line equals the
	 * 			given end point.
	 *       	| new.getEndPoint() == endPoint
	 */
	public Line2D(Point2D startPoint, Point2D endPoint) {
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		this.length = startPoint.distanceTo(endPoint);
	}
	
	/**
	 * Return the start point of this line.
	 */
	public Point2D getStartPoint() {
		return startPoint;
	}
	
	/**
	 * Variable registering the start point of this line.
	 */
	private Point2D startPoint;

	/**
	 * Return the end point of this line.
	 */
	public Point2D getEndPoint() {
		return endPoint;
	}
	
	/**
	 * Variable registering the end point of this line.
	 */
	private Point2D endPoint;
	
	/**
	 * Returns the length of this new line.
	 * 
	 * @return 	The length of this line.
	 * 			| return this.getStartPoint().distanceTo(this.getEndPoint());
	 */
	public Double length() {
		return length;
	}
	
	/**
	 * Variable registering the length of this line.
	 */
	private final double length;
	
	/**
	 * Returns a string representation of this line.
	 * 
	 * @return 	A string representing this line.
	 * 			| result == "[" + this.getStartPoint() + " -> " + this.getEndPoint() + "]"
	 */
	public String toString() {
		return "[" + this.getStartPoint() + " -> " + this.getEndPoint() + "]";
	}

}