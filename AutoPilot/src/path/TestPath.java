package path;

import java.util.ArrayList;
import static org.junit.Assert.*;
import org.junit.*;

import autopilot_vision.Point3D;

public class TestPath {

	@Test
	public void testBla() {
		ArrayList<Point3D> cubes = new ArrayList<Point3D>();
		cubes.add(new Point3D(8,9,-10));
		cubes.add(new Point3D(-8,2,-20));
		cubes.add(new Point3D(8,6,-30));
		cubes.add(new Point3D(-8,8,-40));
		cubes.add(new Point3D(8,2,-50));
		Path path = new Path(new Point3D(0,0,0), cubes);
		System.out.println("1. " + path.cheapestPath(Path.generatePerm(cubes)));
	}
	
	@Test
	public void testBeamSearch() {
		ArrayList<Point3D> cubes = new ArrayList<Point3D>();
		cubes.add(new Point3D(8,9,-10));
		cubes.add(new Point3D(-8,6,-20));
		cubes.add(new Point3D(8,6,-30));
		cubes.add(new Point3D(-8,8,-40));
		cubes.add(new Point3D(8,2,-50));
		cubes.add(new Point3D(-8,5,-60));
		cubes.add(new Point3D(8,7,-70));
		cubes.add(new Point3D(-8,2,-80));
		cubes.add(new Point3D(8,0,-90));
		cubes.add(new Point3D(-8,1,-100));
		cubes.add(new Point3D(8,2,-110));
		cubes.add(new Point3D(-8,9,-120));
		Path path = new Path(new Point3D(0,0,0), cubes);
		System.out.println("2. " + path.beamSearch(cubes));
	}
}
