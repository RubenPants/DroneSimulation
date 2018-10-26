package autopilot_planning;

import autopilot_vision.Cube;
import java.util.*;

public class CubeComparator implements Comparator<Cube>{
	public int compare(Cube a, Cube b) {

		double distanceA = a.getDistance();
		double distanceB = b.getDistance();

		if (distanceA == -1 && distanceB != -1){
			return 1;
		}
		if (distanceA != -1 && distanceB == -1){
			return -1;
		}
		if(distanceA > distanceB) {
			return 1;
		} 
		else if (distanceA == distanceB) { // distanceA == -1 && distanceB == -1 zit hier ook in.
			return 0;
		} 
		else {
			return -1;
		}
	}
}
