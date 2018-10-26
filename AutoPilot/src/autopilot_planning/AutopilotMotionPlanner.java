package autopilot_planning;

import java.util.ArrayList;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;
import org.lwjgl.util.vector.Vector4f;

import interfaces.AutopilotConfig;
import interfaces.AutopilotInputs;
import autopilot_vision.Cube;
import autopilot_vision.Point2D;
import autopilot_vision.Point3D;

/**
 * A class of motion planners for drone autopilots, for determining what direction
 *  the drone should fly towards.
 *  
 * @author Team Saffier
 * @version 1.0
 */
public class AutopilotMotionPlanner {
	
	/**
	 * Initialise this new motion planner.
	 */
	public AutopilotMotionPlanner() {
		// Initialization code
	}
	
	/**
	 * Determine the next move of the autopilot's drone for the given configuration,
	 *  input and list of detected cubes. 
	 * 
	 * @param 	configuration
	 * 			The current configuration of the autopilot.
	 * @param 	inputs
	 * 			The latest inputs the autopilot received.
	 * @param 	cubes
	 * 			A list of cubes that have been detected. This may include cubes
	 * 			that are not visible at this point in time.
	 * @return 	An array of 2 floats. 
	 * 				The first float represents the desired pitch. (In Radians)
	 * 				The second float represents the desired heading. (In Radians)
	 */
	public float[] nextMove(AutopilotConfig configuration, AutopilotInputs inputs, ArrayList<Cube> cubes) {
		
		// Should return requested pitch and heading
		
		Point3D dronePosition = new Point3D(inputs.getX(), inputs.getY(), inputs.getZ());
		BasicPath bPath = new BasicPath(dronePosition, cubes);
		float[] output = new float[2];
		
		float haov = configuration.getHorizontalAngleOfView(), vaov = configuration.getVerticalAngleOfView();
		float cols = configuration.getNbColumns(), rows = configuration.getNbRows();
		
		bPath.sortCubes();
		Cube nextCube = bPath.selectNextCube();
		if (nextCube != null && nextCube.isVisible()) {
			
			Point2D center = nextCube.getCenter();
			
			double horizontal = (center.getX() - cols/2)/(cols/2), vertical = (center.getY() - rows/2)/(rows/2);
			
			// float pitch = (float)Math.toRadians(vertical * vaov / 2);
			// float heading = - (float)Math.toRadians(horizontal * haov / 2);
			float pitch = - (float)(vertical * vaov / 2);
			float heading = - (float)(horizontal * haov / 2);

			output[0] = pitch;
			output[1] = heading;
			return output;
		}
		else if (nextCube != null && nextCube.getLocation() != null && !nextCube.isVisible()){ 
			
			// inverse of atan2
			// hoek van x-as naar z-as.
			int dx = (int) (5 * Math.cos(inputs.getHeading()));
			int dz = (int) (5 * Math.sin(inputs.getHeading()));
			Point3D headingPoint = new Point3D(dronePosition.getX()+dx, dronePosition.getY(), dronePosition.getZ()+dz);
			double horizontal = Path.calculateHorizontalAngle(nextCube.getLocation(), dronePosition, headingPoint);
			
			// hoek van z-as naar y-as.
			int dz2 = (int) (5 * Math.cos(inputs.getPitch()));
			int dy2 = (int) (5 * Math.sin(inputs.getPitch()));
			Point3D pitchPoint = new Point3D(dronePosition.getX(), dronePosition.getY() + dy2, dronePosition.getZ()+dz2);
			double vertical = Path.calculateVerticalAngle(nextCube.getLocation(), dronePosition, pitchPoint);
			
			
			float pitch = - (float)(vertical * vaov / 2);
			float heading = - (float)(horizontal * haov / 2);
			// float pitch = (float)Math.toRadians(Math.toDegrees(vertical) * vaov / 2);
			// float heading = - (float)Math.toRadians(Math.toDegrees(horizontal) * haov / 2);
			
			output[0] = - (float)(1.2 * vertical);
			output[1] = - (float)horizontal;
			
			return output;

		}
		else {
			output[0] = 1000f;
			output[1] = 1000f;
			return output;
		}
		

		
	}
	
	public float[] nextMove2(AutopilotConfig configuration, AutopilotInputs inputs, ArrayList<Cube> cubes) {
		
		// Should return requested pitch and heading
		
		Point3D dronePosition = new Point3D(inputs.getX(), inputs.getY(), inputs.getZ());
		BasicPath bPath = new BasicPath(dronePosition, cubes);
		float[] output = new float[2];
		
		float haov = configuration.getHorizontalAngleOfView(), vaov = configuration.getVerticalAngleOfView();
		float cols = configuration.getNbColumns(), rows = configuration.getNbRows();
		
		bPath.sortCubes();
		Cube nextCube = bPath.selectNextCube();
		if (nextCube != null && nextCube.isVisible()) {
			
			Point2D center = nextCube.getCenter();
			
			double horizontal = (center.getX() - cols/2)/(cols/2), vertical = (center.getY() - rows/2)/(rows/2);
			
			// float pitch = (float)Math.toRadians(vertical * vaov / 2);
			// float heading = - (float)Math.toRadians(horizontal * haov / 2);
			float pitch = - (float)(vertical * vaov / 2);
			float heading = - (float)(horizontal * haov / 2);

			output[0] = pitch;
			output[1] = heading;
			return output;
		}
		
		else if (nextCube != null && nextCube.getLocation() != null && !nextCube.isVisible()){ 
			float pitchReq, headingReq;
			float droneX = inputs.getX();
			float droneY = inputs.getY();
			float droneZ = inputs.getZ();
			Matrix4f toDrone = getWorldToDroneTransformationMatrix(inputs.getHeading(), inputs.getPitch(), inputs.getRoll());
						
					Point3D startCubeCoor = nextCube.getLocation();
					Vector3f requestedVector = new Vector3f((float) (startCubeCoor.getX() - droneX), (float) (startCubeCoor.getY() - droneY), (float) (startCubeCoor.getZ() - droneZ));
										
					Vector3f forwardVector = transformVector(toDrone, requestedVector);
					Vector3f headingVector = new Vector3f(forwardVector.x, 0, forwardVector.z).normalise(null);
						
					headingReq = (float) (Math.atan2(-headingVector.x, -headingVector.z));
					pitchReq = (float) (Math.atan2(forwardVector.y, Vector3f.dot(forwardVector, headingVector)));

			if (nextCube != null) {
				output[0] = pitchReq;
				output[1] = headingReq;
				return output;
			}
		}

			output[0] = 1000f;
			output[1] = 1000f;
			return output;
		

		
	}
	
	private Vector3f transformVector(Matrix4f matrix, Vector3f vector) {
		Vector4f vector4 = new Vector4f(vector.x, vector.y, vector.z, 1);
		Matrix4f.transform(matrix, vector4, vector4);
		
		return new Vector3f(vector4.x, vector4.y, vector4.z);
	}

	private Matrix4f getWorldToDroneTransformationMatrix(float heading, float pitch, float roll) {
		return Matrix4f.invert(getDroneToWorldTransformationMatrix(heading, pitch, roll), null);
	}

	private Matrix4f getDroneToWorldTransformationMatrix(float heading, float pitch, float roll) {
		Matrix4f matrix = new Matrix4f();
		matrix.setIdentity();
		
		Matrix4f.rotate((float) heading, new Vector3f(0, 1, 0), matrix, matrix);
		Matrix4f.rotate((float) pitch, new Vector3f(1, 0, 0), matrix, matrix);
		Matrix4f.rotate((float) roll, new Vector3f(0, 0, 1), matrix, matrix);
		
		return matrix;
	}
	
}