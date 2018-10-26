package autopilot_planning;

import java.util.*;

import autopilot_vision.Cube;
import autopilot_vision.Point2D;
import autopilot_vision.Point3D;

public class BasicPath {

	// TODO: ORDE!
	/**
	 * Initialize a basic path with the given position of the drone and the given list of cubes.
	 * 
	 * @param	dronePosition
	 * 			The position of the drone.
	 * @param 	cubes
	 * 			The list of cubes of this basic path.
	 */
	public BasicPath(Point3D dronePosition, List<Cube> cubes) {
		setDronePosition(dronePosition);
		setCubes(cubes);
		setForward(true);
	}
	
	/**
	 * Set the cubes of this basic path to the given list of cubes.
	 * 
	 * @param 	cubes
	 * 			The given list of cubes.
	 */
	private void setCubes(List<Cube> cubes) {
		this.cubes = cubes;
	}
	
	/**
	 * Set the position of the drone to the given position.
	 * 
	 * @param 	dronePosition
	 * 			The given position of the drone.
	 */
	private void setDronePosition(Point3D dronePosition){
		this.dronePosition = dronePosition;
	}
	
	/**
	 * Variable referencing the position of the drone.
	 */
	private Point3D dronePosition;
	
	/**
	 * Return the position of the drone.
	 * 
	 * @return	dronePosition
	 */
	public Point3D getPosition(){
		return this.dronePosition;
	}
	
	/**
	 * Return the list of cubes of this path.
	 * 
	 * @return	cubes
	 */
	public List<Cube> getCubes(){
		return this.cubes;
	}
	
	/**
	 * Variable referencing to the list of cubes of this basic path.
	 */
	private List<Cube> cubes;
	
	/**
	 * Variable referencing the direction in which the drone is flying.
	 */
	private boolean forward = true;
	
	/**
	 * Check whether this drone is flying forward or not.
	 * 
	 * @return	forward
	 */
	public boolean isForward() {
		return forward;
	}
	
	/**
	 * Set forward to the given flag.
	 * 
	 * @param 	forward
	 * 			The given flag.
	 */
	private void setForward(boolean forward) {
		this.forward = forward;
	}
	
	/**
	 * Sorts the cubes of this basic path.
	 * 
	 * @return Sorted list of cubes.
	 */
	public List<Cube> sortCubes(){
		// TODO: misschien overbodige if
		if (!(this.getCubes().isEmpty())) {
			List<Cube> listCubes = getCubes();
			Collections.sort(listCubes, new CubeComparator());
			setCubes(listCubes);
		}
		return this.getCubes();
	}
	
	// TODO de lijst van kubussen moet zeker regelmatig upgedatet worden, anders werkt dit niet
	/**
	 * Selects next cube to visit, ignores unreachable cubes.
	 * 
	 * @return next Cube to visit.
	 */
	public Cube selectNextCube(){
		int i = 0;
		if (getCubes().isEmpty()) {
			return null;
		}
		while (i < this.cubes.size()) {
			if (isReachable(getCubes().get(i))){
				return getCubes().get(i);
			}
			else{
				i += 1;
			}
		}
		setForward(!isForward());
		return getCubes().get(0);
	}
	
	/**
	 * Check whether the given cube is reachable.
	 * 
	 * @param 	cube
	 * 			The cube to check if it's reachable.
	 * @return	False if the cube is behind the drone, true otherwise.
	 */
	public boolean isReachable(Cube cube){
		if (cube.getLocation() == null){
			return false;
		}
		else if (isForward() && cube.getLocation().getZ() > getPosition().getZ() ){
			return false;
		}
		else if (!isForward() && cube.getLocation().getZ() < getPosition().getZ()) {
			return false;
		}
		// TODO elseif(minimumstraal){negeren} --> niet mogelijk, want we hebben geen snelheid
		// rekening houden met de richting naar waar de drone kijkt gaat ook niet, dan verlies je
		// het voordeel van deze methode dom te houden
		return true;
	}
	
	/**
	 * Return the center of the closest cube (the first one in the list).
	 * 
	 * @return	The closest center, null if list is empty.
	 */
	public Point2D closestCenter() {
		if (getCubes().isEmpty()) {
			return null;
		}
		return getCubes().get(0).getCenter();
	}
}


