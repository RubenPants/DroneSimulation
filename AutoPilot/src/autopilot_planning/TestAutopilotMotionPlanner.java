package autopilot_planning;

import static org.junit.Assert.*;
import org.junit.*;
import java.util.*;

import autopilot_vision.Cube;
import autopilot_vision.Point3D;
import autopilot_vision.Rectangle2D;

public class TestAutopilotMotionPlanner {
	
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
	public void testCalculateHorizontalAngle(){
		path = new BasicPath(new Point3D(0,0,10), cubes);
		path.sortCubes();
		Cube nextCube = path.selectNextCube();
		Point3D headingPoint = new Point3D(0, 0, 0);
		double horizontal = Path.calculateHorizontalAngle(nextCube.getLocation(), new Point3D(0,0,0), headingPoint);
		assertTrue(horizontal==0);
	}
	
	@Test
	public void testNullLocation(){
		path = new BasicPath(new Point3D(0,0,0), nullCube);
		path.sortCubes();
		Cube nextCube = path.selectNextCube();
		Point3D headingPoint = new Point3D(0, 5, -12);
		
		assertTrue(true);
	}
}
