package autopilot_planning;

import java.util.ArrayList;
import static org.junit.Assert.*;
import org.junit.*;
import java.util.*;

import autopilot_vision.Cube;
import autopilot_vision.Point3D;
import autopilot_vision.Rectangle2D;

public class TestBasicPath {

	Cube cube1 = new Cube(new Rectangle2D(20,30,74,84), 12, 2);
	Cube cube2 = new Cube(new Rectangle2D(56,76,47,67), 5, 71);
	Cube cube3 = new Cube(new Rectangle2D(36,66,66,96), 33, 42);
	ArrayList<Cube> cubes = new ArrayList<Cube>(Arrays.asList(cube1, cube2, cube3));
	Cube cubeNull = new Cube(new Rectangle2D(36,66,66,96), 33, 42);
	ArrayList<Cube> nullCube = new ArrayList<Cube>(Arrays.asList(cubeNull));
	BasicPath path = new BasicPath(new Point3D(0,0,0), cubes);
	
	@Before
	public void fixture() {
		cube1.setDistance(235);
		cube2.setDistance(108);
		cube3.setDistance(35);
		cubeNull.setDistance(-1);
		cube1.setLocation(new Point3D(3,1,-15));
		cube2.setLocation(new Point3D(-2,2,-10));
		cube3.setLocation(new Point3D(1,3,-5));
		cubeNull.setLocation(null);
	}
	
	@Test
	public void testInitializer() {
		assertTrue(path.getPosition().equalsTo(new Point3D(0,0,0)));
		assertTrue(path.getCubes() == cubes);
		assertTrue(path.isForward());
	}
	
	@Test
	public void testSortCubes() {
		assertTrue(path.sortCubes().get(0) == cube3);
		assertTrue(path.sortCubes().get(1) == cube2);
		assertTrue(path.sortCubes().get(2) == cube1);
	}
	
	@Test
	public void testClosestCenter() {
		assertTrue(path.closestCenter().equalsTo(cube1.getCenter()));
	}
	
	@Test
	public void testSelectNextCube1() {
		path.sortCubes();
		assertTrue(path.selectNextCube() == cube3);
	}
	
	@Test
	public void testSelectNextCube2() {
		path.sortCubes();
		path = new BasicPath(new Point3D(0,0,-20), cubes);
		path.selectNextCube();
		assertFalse(path.isForward());
	}
	
	@Test
	public void testIsReachable() {
		assertTrue(path.isReachable(cube1));
		path = new BasicPath(new Point3D(0,0,-20), cubes);
		// cube achter drone als forward true
		assertFalse(path.isReachable(cube1));
		path.selectNextCube();
		Cube cube4 = new Cube(new Rectangle2D(36,66,66,96), 33, 42);
		cube4.setDistance(42);
		cube4.setLocation(new Point3D(-1,4,-25));
		// cube "voor" drone, maar drone vliegt terug, dus cube in echt wel achter drone
		assertFalse(path.isReachable(cube4));
	}

	

	
	

}
