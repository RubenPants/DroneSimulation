package autopilot_vision;

import java.util.ArrayList;

/**
 * A class of cubes with at least a 2-dimensional center point and a bounding box,
 * 	both referring to the image in which the cube was detected.
 * 
 * The cube can also have a list of 2-dimensional lines, corresponding to its detected edges.
 * 
 * @author Team Saffier
 * @version 1.0
 */
public class Cube {

	/**
	 * Initialize this new cube with given center point and bounding box.
	 * 
	 * @param	boundingBox
	 * 			The bounding box for this new cube.
	 * @param	hue
	 * 			The hue of the color of this new cube.
	 * @param	saturation
	 * 			The saturation of the color of this new cube.
	 * @post		The new cube's center point equals the given one.
	 * 			| new.getCenter() == center
	 * @post		The new cube's bounding box equals the given one.
	 * 			| new.getBoundingBox() == boundingBox
	 */
	public Cube(Rectangle2D boundingBox, float hue, float saturation) {
		this.boundingBox = boundingBox;
		this.hue = hue;
		this.saturation = saturation;
		this.distance = -1.0;
		this.location = null;
	}
	
	/**
	 * Return the center point of this cube.
	 */
	public Point2D getCenter() {
		return getBoundingBox().getCenter();
	}
	
	/**
	 * Return the bounding box of this cube.
	 */
	public Rectangle2D getBoundingBox() {
		return boundingBox;
	}
	
	/**
	 * Set the bounding box of this cube.
	 */
	public void setBoundingBox(Rectangle2D boundingBox) {
		this.boundingBox = boundingBox;
	}
	
	
	/**
	 * Variable registering the bounding box of the color of this cube.
	 */
	private Rectangle2D boundingBox;
	
	/**
	 * Returns the hue of the color of this cube.
	 */
	public float getHue() {
		return hue;
	}
	
	/**
	 * The hue of this cube.
	 */
	private float hue;
	
	/**
	 * Returns the saturation of the color of this cube.
	 */
	public float getSaturation() {
		return saturation;
	}
	
	/**
	 * The saturation of the color of this cube.
	 */
	private float saturation;
	
	/**
	 * Returns the location of this cube, or null if no location was recorded/approximated.
	 */
	public Point3D getLocation() {
		return location;
	}
	
	/**
	 * Set the location of this cube in 3-dimensional space.
	 * 
	 * @param 	location
	 * 			The new location for this cube.
	 */
	public void setLocation(Point3D location) {
		this.location = location;
	}
	
	/**
	 * Variable registering the location of this cube in 3-dimensional space.
	 * 	This can be an approximation.
	 */
	private Point3D location;
	
	/**
	 * Returns the current distance to this cube, or null if no distance was recorded/approximated.
	 */
	public double getDistance() {
		return distance;
	}
	
	/**
	 * Set the distance to this cube in 3-dimensional space.
	 * 
	 * @param 	distance
	 * 			The new distance for this cube.
	 */
	public void setDistance(double distance) {
		this.distance = distance;
	}
	
	/**
	 * Variable registering the current (approximate) distance to the cube.
	 * 	This can be an approximation.
	 */
	private double distance;
	
	/**
	 * Variable registering whether or not the distance approximation uses the bounding box
	 *  instead of real edges.
	 */
	public boolean distanceUsesBoundingBox;
	
	/**
	 * Returns whether or not this cube is currently visible.
	 */
	public boolean isVisible() {
		return visible;
	}
	
	/**
	 * Set the current visibility of this cube.
	 */
	public void setVisible(boolean visible) {
		this.visible = visible;
	}
	
	/**
	 * Variable denoting whether or not this cube is currently visible.
	 */
	private boolean visible;
	
	/**
	 * Get the number of edges attributed to this cube.
	 */
	public int getNbEdges() {
		return edges.size();
	}
	
	/**
	 * Get the edge at the given index in this cube's list of attributed edges.
	 * 
	 * @param 	index
	 * 			The index of the edge that is wanted.
	 * @return	The edge at given index or null if the index is invalid.
	 */
	public Line2D getEdgeAtIndex(int index) {
		if (index < 0 || index >= edges.size())
			return null;
		return edges.get(index);
	}
	
	/**
	 * Attribute the given edge to this cube.
	 * 
	 * @param	edge
	 * 			The edge that is to be added.
	 */
	public void addEdge(Line2D edge) {
		edges.add(edge);
	}
	
	/**
	 * Remove all edges from this cube's edges list.
	 */
	public void removeAllEdges() {
		edges.clear();
	}
	
	/**
	 * A list of edges attributed to this cube.
	 */
	private ArrayList<Line2D> edges = new ArrayList<Line2D>();
	
	/**
	 * Returns a textual representation of this cube.
	 * 
	 * @return	A string representing this cube.
	 * 			| "[" + getCenter() + " with hue = " + getHue() + " and saturation = " + getSaturation() + "]"
	 */
	public String toString() {
		return "[" + getBoundingBox() + " with hue = " + String.format("%.03f", getHue()) + " and saturation = " +String.format("%.03f", getSaturation()) + " (visible = " + isVisible() + " at distance = " + getDistance() + " and location = " + getLocation() + ")]";
	}
	
}