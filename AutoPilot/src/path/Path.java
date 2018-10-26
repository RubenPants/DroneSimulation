package path;

import java.util.*;

import autopilot_vision.Point2D;
import autopilot_vision.Point3D;

public class Path {
	
	/**
	 * Initialize a path for the drone with given position and the given cubes to go to.
	 * 
	 * @param 	position
	 * 			The position of the drone.
	 * @param 	cubes
	 * 			The positions of all the cubes.
	 */
	public Path(Point3D position, List<Point3D> cubes) {
		setCubes(cubes);
		setPosition(position);
	}
	
	/**
	 * All the cubes that have to be reached.
	 */
	private List<Point3D> cubes;
	
	/**
	 * The position of the drone.
	 */
	private Point3D position;
	
	/**
	 * Set the positions of the cubes to the given ArrayList 'cubes'.
	 * 
	 * @param 	cubes
	 * 			The positions of all the cubes.
	 */
	private void setCubes(List<Point3D> cubes) {
		this.cubes = cubes;
	}
	
	/**
	 * Returns an ArrayList with all the positions of the cubes.
	 */
	public List<Point3D> getCubes(){
		return this.cubes;
	}
	
	/**
	 * Set the position of this drone to the given position.
	 * 
	 * @param 	position
	 * 			The position of the drone.
	 */
	private void setPosition(Point3D position) {
		this.position = position;
	}
	
	/**
	 * Returns the position of the drone.
	 */
	public Point3D getPosition(){
		return this.position;
	}
	
	/**
	 * Calculates the horizontal angle formed by the 3 given points, in that order.
	 * 
	 * @param 	pos1
	 * 			The point where the first line of the angle starts.
	 * @param 	pos2
	 * 			The point where the first line ends and the second line starts. This is also the
	 * 			point where the angle is.
	 * @param 	pos3
	 * 			The point where the second line ends.
	 * @return	The arccos of the dotproduct of both vectors divided by the product of their
	 * 			lengths.
	 */
	public static double calculateHorizontalAngle(Point3D pos1, Point3D pos2, Point3D pos3) {
		if (pos1.equalsTo(pos2) || pos1.equalsTo(pos3) || pos2.equalsTo(pos3)) {
			return 0;
		}
		Point2D AB = new Point2D((int) (pos2.getX() - pos1.getX()), (int) (pos2.getZ() - pos1.getZ()));
		Point2D BC = new Point2D((int) (pos3.getX() - pos2.getX()), (int) (pos3.getZ() - pos2.getZ()));
		Point2D p1 = new Point2D((int) pos1.getX(), (int) pos1.getZ());
		Point2D p2 = new Point2D((int) pos2.getX(), (int) pos2.getZ());
		Point2D p3 = new Point2D((int) pos3.getX(), (int) pos3.getZ());
		double dotProduct = AB.getX()*BC.getX() + AB.getY()*BC.getY();
		return Math.acos(dotProduct/ (p1.distanceTo(p2) * p2.distanceTo(p3)));
	}
	
	/**
	 * Calculates the vertical angle formed by the 3 given points, in that order.
	 * 
	 * @param 	pos1
	 * 			The point where the first line of the angle starts.
	 * @param 	pos2
	 * 			The point where the first line ends and the second line starts. This is also the
	 * 			point where the angle is.
	 * @param 	pos3
	 * 			The point where the second line ends.
	 * @return	The arccos of the dotproduct of both vectors divided by the product of their
	 * 			lengths.
	 */
	public static double calculateVerticalAngle(Point3D pos1, Point3D pos2, Point3D pos3) {
		if (pos1.equalsTo(pos2) || pos1.equalsTo(pos3) || pos2.equalsTo(pos3)) {
			return 0;
		}
		Point2D AB = new Point2D((int) (pos2.getY() - pos1.getY()), (int) (pos2.getZ() - pos1.getZ()));
		Point2D BC = new Point2D((int) (pos3.getY() - pos2.getY()), (int) (pos3.getZ() - pos2.getZ()));
		Point2D p1 = new Point2D((int) pos1.getY(), (int) pos1.getZ());
		Point2D p2 = new Point2D((int) pos2.getY(), (int) pos2.getZ());
		Point2D p3 = new Point2D((int) pos3.getY(), (int)pos3.getZ());
		double dotProduct = AB.getX()*BC.getX() + AB.getY()*BC.getY();
		return Math.acos(dotProduct/ (p1.distanceTo(p2) * p2.distanceTo(p3)));
	}
	
	/**
	 * Calculates the horizontal angle between the starting position of the drone and the given
	 * point.
	 * 
	 * @param 	pos
	 * 			The point needed to calculate the angle.
	 * @return	The horizontal angle from the starting position.
	 * 			| calculateHorizontalAngle(new Point3D(0,0,-1), this.getPosition(), pos)
	 */
	private double calculateStartAngleHorizontal(Point3D pos) {
		return calculateHorizontalAngle(new Point3D(0,0,-1), new Point3D(0,0,0), pos)
				- calculateHorizontalAngle(new Point3D(0,0,-1), new Point3D(0,0,0), this.getPosition());
	}
	
	/**
	 * Calculates the vertical angle between the starting position of the drone and the given
	 * point.
	 * 
	 * @param 	pos
	 * 			The point needed to calculate the angle.
	 * @return	The vertical angle from the starting position.
	 * 			| calculateVerticalAngle(new Point3D(0,0,-1), this.getPosition(), pos)
	 */
	private double calculateStartAngleVertical(Point3D pos) {
		return calculateVerticalAngle(new Point3D(0,0,-1), new Point3D(0,0,0), pos)
				- calculateVerticalAngle(new Point3D(0,0,-1), new Point3D(0,0,0), this.getPosition());
	}
	
	/**
	 * Calculates the cost for going from the starting position of the drone to a cube.
	 * 
	 * @param 	pos
	 * 			The position of the cube.
	 * @return	The cost.
	 */
	private double startCost(Point3D pos) {
		double distanceFactor = 1;
		double horAngleFactor = 50;
		double verAngleFactor = 6;
		double distance = this.getPosition().distanceTo(pos);
		//if (pos.getY() < this.getPosition().getY() && distance > 45) {
		//	verAngleFactor = 12;
		//}
		double horAngle = calculateStartAngleHorizontal(pos);
		if (horAngle == 0) {
			horAngleFactor = 10000;
			horAngle = 1;
		}
		double verAngle = calculateStartAngleVertical(pos);
		if (verAngle == 0) {
			verAngleFactor = 10000;
			verAngle = 1;
		}
		//System.out.println(distance + " distancecost (start) " + distanceFactor * distance);
		//System.out.println(horAngle + " horAnglecost (start) " + horAngle * horAngleFactor);
		//System.out.println(verAngle + " verAnglecost (start) " + verAngle * verAngleFactor);
		return distanceFactor * distance + horAngle * horAngleFactor + verAngle * verAngleFactor;
	}
	
	/**
	 * Calculates the cost for going from one cube to another.
	 * 
	 * @param 	current
	 * 			The cube to go to.
	 * @param 	last
	 * 			The cube where you came from.
	 * @param 	secondLast
	 * 			The cube or position before that.
	 * @return	The cost.
	 */
	private double restCost(Point3D current, Point3D last, Point3D secondLast) {
		double distanceFactor = 1;
		double horAngleFactor = 30;
		double verAngleFactor = 1.3;
		double distance = last.distanceTo(current);
		//if (current.getY() < last.getY() && distance > 45) {
		//	verAngleFactor = 2.5;
		//}
		double horAngle = calculateHorizontalAngle(secondLast, last, current);
		if (horAngle == 0) {
			horAngleFactor = 10000;
			horAngle = 1;
		}
		double verAngle = calculateVerticalAngle(secondLast, last, current);
		if (verAngle == 0) {
			verAngleFactor = 10000;
			verAngle = 1;
		}
		//System.out.println(distance + " distancecost " + distanceFactor * distance);
		//System.out.println(horAngle + " horAnglecost " + horAngleFactor / horAngle);
		//System.out.println(verAngle + " verAnglecost " + verAngleFactor / verAngle);
		return distanceFactor * distance + horAngleFactor / horAngle + verAngleFactor / verAngle;
	}
	
	/**
	 * Calculates the total cost for a given path.
	 * 
	 * @param 	path
	 * 			An arrayList with the path from which the cost needs to be calculated.
	 * @return	The total cost.
	 */
	private double totalCost(List<Point3D> path) {
		double cost = startCost(path.get(0)) + restCost(path.get(1), path.get(0), this.getPosition());
		for (int i=2; i < path.size(); i++) {
			cost += restCost(path.get(i), path.get(i-1), path.get(i-2));
		}
		return cost;
	}
	
	/**
	 * Generates all the possible permutations for the given list of cubes.
	 * 
	 * @param 	original
	 * 			The original given list of cubes.
	 * @return	A list with all the permutations.
	 */
	public static <Point3D> List<List<Point3D>> generatePerm(List<Point3D> original) {
	     if (original.size() == 0) {
	       List<List<Point3D>> result = new ArrayList<List<Point3D>>(); 
	       result.add(new ArrayList<Point3D>()); 
	       return result; 
	     }
	     Point3D firstElement = original.remove(0);
	     List<List<Point3D>> returnValue = new ArrayList<List<Point3D>>();
	     List<List<Point3D>> permutations = generatePerm(original);
	     for (List<Point3D> smallerPermutated : permutations) {
	       for (int index=0; index <= smallerPermutated.size(); index++) {
	         List<Point3D> temp = new ArrayList<Point3D>(smallerPermutated);
	         temp.add(index, firstElement);
	         returnValue.add(temp);
	       }
	     }
	     return returnValue;
	   }
	
	/**
	 * Finds the cheapest path of all the given paths.
	 * 
	 * @param 	allPaths
	 * 			All the paths from which you have to find the cheapest one.
	 * @return	The cheapest path.
	 */
	public List<Point3D> cheapestPath(List<List<Point3D>> allPaths){
		double cost = totalCost(allPaths.get(0));
		List<Point3D> currentCheapestPath = allPaths.get(0);
		Iterator<List<Point3D>> iterator = allPaths.iterator();
		while (iterator.hasNext()) {
			List<Point3D> current = iterator.next();
			double totalCost = totalCost(current);
			//System.out.println(current + ": cost = " + totalCost);
			if (totalCost < cost) {
				cost = totalCost;
				currentCheapestPath = current;
			}
		}
		return currentCheapestPath;
	}
	
	/**
	 * Return the best path according to (an adapted version of) beam search.
	 * 
	 * @param 	original
	 * 			The list of cubes to visit.
	 * @return	The best path.
	 */
	public List<Point3D> beamSearch(List<Point3D> original){
		int width = 10;
		List<List<Point3D>> paths = new ArrayList<List<Point3D>>();
		if (original.size() <= width) {
			for (Point3D point: original) {
				List<Point3D> temp = new ArrayList<Point3D>();
				temp.add(point);
				paths.add(temp);
			}
		}
		else {
			Iterator<Point3D> iterator = original.iterator();
			while (iterator.hasNext()) {
				Point3D current = iterator.next();
				if (paths.size() < width) {
					List<Point3D> temp = new ArrayList<Point3D>();
					temp.add(current);
					paths.add(temp);
				}
				else {
					Point3D max = paths.get(0).get(0);
					double maxCost = startCost(max);
					int index = 0;
					for (int j = 1; j<width; j++) {
						double cost = startCost(paths.get(j).get(0));
						if (cost > maxCost) {
							max = paths.get(j).get(0);
							maxCost = startCost(max);
							index = j;
						}
					}
					if (startCost(current) < maxCost) {
						paths.remove(index);
						List<Point3D> temp = new ArrayList<Point3D>();
						temp.add(current);
						paths.add(temp);
					}
				}
			}
		}
		while (paths.get(0).size() < original.size()) {
			Iterator<List<Point3D>> iterator1 = paths.iterator();
			List<List<Point3D>> newPaths = new ArrayList<List<Point3D>>();
			while (iterator1.hasNext()) {
				List<Point3D> currentList = iterator1.next();
				Iterator<Point3D> iterator2 = original.iterator();
				while (iterator2.hasNext()) {
					Point3D currentPoint = iterator2.next();
					if (!currentList.contains(currentPoint)) {
						List<Point3D> newList = new ArrayList<Point3D>();
						for (Point3D point: currentList) {
							newList.add(point);
						}
						newList.add(currentPoint);
						newPaths.add(newList);
					}
				}
			}
			Iterator<List<Point3D>> iterator3 = newPaths.iterator();
			paths.clear();
			paths.add(iterator3.next());
			while (iterator3.hasNext()) {
				List<Point3D> currentNewPaths = iterator3.next();
				double currentCost = totalCost(currentNewPaths);
				if ((paths.size() == width && currentCost < totalCost(paths.get(paths.size()-1)) || paths.size() < width)) {
					if (paths.size() == width) {
						paths.remove(paths.size()-1);
					}
					Iterator<List<Point3D>> iterator4 = paths.iterator();
					boolean added = false;
					while (!added && iterator4.hasNext()) {
						List<Point3D> currentPaths = iterator4.next();
						double currentCurrentCost = totalCost(currentPaths);
						if (currentCost >= currentCurrentCost) {
							paths.add(paths.indexOf(currentPaths)+1, currentNewPaths);
							added = true;
						}
					}
					if (!added) {
						paths.add(0, currentNewPaths);
					}
				}
			}
		}
		return paths.get(0);
	}
}