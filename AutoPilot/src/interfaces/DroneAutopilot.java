package interfaces;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JCheckBox;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;
import org.lwjgl.util.vector.Vector4f;

import autopilot_gui.AutopilotGUI;
import autopilot_physics.AutopilotPhysics;
import autopilot_planning.AutopilotMotionPlanner;
import autopilot_vision.AutopilotImageAnalyser;
import autopilot_vision.Cube;
import autopilot_vision.Point2D;
import autopilot_vision.Point3D;

/**
 * A class of autopilots for steering a drone. Autopilots translate a drone's configuration and
 *  constantly updated situation to output commands, such that that very drone can reach its 
 *  desired destination.
 * 
 * @author 	Team Saffier
 * @version 	1.0
 */
public class DroneAutopilot implements Autopilot {
		
	/**
	 * Initialise this autopilot with given image analyser.
	 * 
	 * @param	analyser
	 * 			The image analyser for this new autopilot.
	 * @post		This autopilot's image analyser equals the given one.
	 * 			| new.getAnalyser() == analyser
	 * @throws	IllegalArgumentException
	 * 			The given image analyser is invalid.
	 * 			| analyser == null
	 */
	public DroneAutopilot(AutopilotImageAnalyser analyser) throws IllegalArgumentException {
		if (analyser == null)
			throw new IllegalArgumentException("Invalid image anlyser");
		this.analyser = new AutopilotImageAnalyser();
		this.planner = new AutopilotMotionPlanner();
//		this.gui = new AutopilotGUI();
//		this.gui.addActionListener(new ActionListener() {
//			public void actionPerformed(ActionEvent e) {
//				JCheckBox box = ((JCheckBox)e.getSource());
//				DroneAutopilot.this.usesPlanner = box.isSelected();
//			}
//		});
	}
	
	/**
	 * Initialise this autopilot.
	 * 
	 * @effect This autopilot is initialised with a new image analyser.
	 */
	public DroneAutopilot() {
		this(new AutopilotImageAnalyser());
	}
	
	/**
	 * Variable referencing an image analyser.
	 */
	private AutopilotImageAnalyser analyser;
	
	/**
	 * Variable referencing a motion planner.
	 */
	private AutopilotMotionPlanner planner;
	
	/**
	 * Variable referencing a gui for this autopilot.
	 */
	private AutopilotGUI gui;
	
	/**
	 * Set this autopilot's configuration to the given one.
	 * 
	 * @param	configuration
	 * 			The new configuration for this autopilot.
	 */
	public void setConfiguration(AutopilotConfig configuration) {
		if (configuration == null)
			return;
		this.configuration = configuration;
	}
	
	/**
	 * Returns this autopilot's current configuration.
	 */
	public AutopilotConfig getConfiguration() {
		return this.configuration;
	}
	
	/**
	 * This autopilot's current configuration.
	 */
	private AutopilotConfig configuration;
	
	/**
	 * This autopilot's physics engine.
	 */
	private AutopilotPhysics physics;
	
	/**
	 * Returns whether or not this autopilot uses motion planning.
	 */
	public boolean getUsesPlanner() {
		return usesPlanner;
	}
	
	/**
	 * Set whether or not this autopilot uses motion planning.
	 * 
	 * @param 	usesPlanner
	 * 			True if and only if the planner is to be used.
	 */
	public void setUsesPlanner(boolean usePlanner) {
		this.usesPlanner = usePlanner;
	}
	
	/**
	 * Variable registering whether or not motion planning is applied.
	 */
	private boolean usesPlanner = false;
	
	/**
	 * Let this autopilot's image analyser analyse the given input.
	 * 
	 * @param 	inputs
	 * 			The autopilot input that is to be analysed.
	 * @return	The results of the analysis by the image analyser and the physical model.
	 */
	private AutopilotOutputs analyseInputs(AutopilotInputs inputs, boolean firstFrame) {
		
		if (firstFrame)
			physics.getInputsStart(inputs);
		else
			physics.getInputs(inputs);
			
		ArrayList<Cube> cubes = analyser.locateAllCubes(configuration, inputs, true);
		int amountOfCubes = cubes.size();
		
		System.out.println("DETECTED CUBES [");
		for (Cube cube : cubes)
			System.out.println(cube);
		System.out.println("]");
			
		Cube closestCube = null;
		Point3D currentPosition = new Point3D(inputs.getX(), inputs.getY(), inputs.getZ());
		double minDistance = 0, currentDistance;
		float pitchReq, headingReq;
		float pitchReqMin = 0, headingReqMin = 0;
		float precisionAngle = (float)(Math.PI/12);
		float liftSlope = configuration.getWingLiftSlope();
		float droneX = inputs.getX();
		float droneY = inputs.getY();
		float droneZ = inputs.getZ();
		Matrix4f toDrone = getWorldToDroneTransformationMatrix(inputs.getHeading(), inputs.getPitch(), inputs.getRoll());
		
		for (Cube cube : cubes) {
			if (cube.getLocation() != null) {
				currentDistance = cube.getLocation().distanceTo(currentPosition);
					
				Point3D startCubeCoor = cube.getLocation();
				Vector3f requestedVector = new Vector3f((float) (startCubeCoor.getX() - droneX), (float) (startCubeCoor.getY() - droneY), (float) (startCubeCoor.getZ() - droneZ));
									
				Vector3f forwardVector = transformVector(toDrone, requestedVector);
				Vector3f headingVector = new Vector3f(forwardVector.x, 0, forwardVector.z).normalise(null);
					
				headingReq = (float) (Math.atan2(-headingVector.x, -headingVector.z));
				pitchReq = (float) (Math.atan2(forwardVector.y, Vector3f.dot(forwardVector, headingVector)));

				if (liftSlope <= 0.1)
					precisionAngle = (float) (Math.PI/24);
				else if (liftSlope >= 0.4 || currentDistance > 80)
					precisionAngle = (float) (Math.PI/6);
				else if (liftSlope >= 0.3 || currentDistance > 50)
					precisionAngle = (float) (Math.PI/9);
				
				// If there's a visible cube, prioritize it
				// Of all visible cubes, that the one that's closest
				if ((closestCube == null || currentDistance < minDistance || (!closestCube.isVisible() && cube.isVisible()))
					&& !(closestCube != null && closestCube.isVisible() && !cube.isVisible())
					&& (Math.sqrt(Math.pow(headingReq, 2) + 2*Math.pow(pitchReq, 2)) < precisionAngle)) {
					if ((Math.abs(currentDistance - minDistance) > 5)
							&& (minDistance < 95 || currentDistance < 95)) {
						pitchReqMin = pitchReq;
						headingReqMin = headingReq;
						closestCube = cube;
						minDistance = currentDistance;
					} else if (headingReqMin < headingReq) {
						pitchReqMin = pitchReq;
						headingReqMin = headingReq;
						closestCube = cube;
						minDistance = currentDistance;
					}
				}
			}
		}
			
		if (closestCube != null) {
			return physics.output((float) (pitchReqMin), (float) (headingReqMin), true, amountOfCubes);	
		}
			
		else {
			cubes = analyser.locateAllCubes(configuration, inputs, false);
				
			// No cubes in history --> Search for cubes
			if (cubes.size() > 0) {						
				float haov = configuration.getHorizontalAngleOfView(), vaov = configuration.getVerticalAngleOfView();
				float cols = configuration.getNbColumns(), rows = configuration.getNbRows();
					
				Point2D startCube = cubes.get(0).getCenter();
					
				double minHorizontal = (startCube.getX() - cols/2)/(cols/2);
				double minVertical = (startCube.getY() - rows/2)/(rows/2);
					
				Point2D newCubeCoor;
				int cubeInt;
					
				double horizontal, vertical;
				float pitch, heading;
			
				Cube cube = cubes.get(0);
				for (cubeInt = 1; cubeInt < cubes.size(); cubeInt++) {
					newCubeCoor = cubes.get(cubeInt).getCenter();
					horizontal = (newCubeCoor.getX() - cols/2)/(cols/2);
					vertical = (newCubeCoor.getY() - rows/2)/(rows/2);
						
					if ((Math.pow(horizontal, 2) + 2*Math.pow(vertical, 2)) < (Math.pow(minHorizontal, 2) + 2*Math.pow(minVertical, 2))) {
						minHorizontal = horizontal;
						minVertical = vertical;
						cube = cubes.get(cubeInt);
					}
				}
					
				pitch = - (float)(minVertical * vaov / 2);
				heading = - (float)(minHorizontal * haov / 2);
				return physics.output((float)pitch, (float)heading, true, amountOfCubes);
			}
			
			else {
				cubes = analyser.locateAllCubes(configuration, inputs, true);
				if (cubes.size() > 0) {						
					Point3D startCubeCoor = cubes.get(0).getLocation();
					if (startCubeCoor == null) {
						return physics.output(1000f, 1000f, false, amountOfCubes);				
					}

					Vector3f requestedVector = new Vector3f((float) (startCubeCoor.getX() - droneX), (float) (startCubeCoor.getY() - droneY), (float) (startCubeCoor.getZ() - droneZ));
											
					Vector3f forwardVector = transformVector(toDrone, requestedVector);
					Vector3f headingVector = new Vector3f(forwardVector.x, 0, forwardVector.z).normalise(null);
						
					headingReqMin = (float) (Math.atan2(-headingVector.x, -headingVector.z));
					pitchReqMin = (float) (Math.atan2(forwardVector.y, Vector3f.dot(forwardVector, headingVector)));
						
					int cubeInt;
					Point3D newCubeCoor;
					for (cubeInt = 1; cubeInt < cubes.size(); cubeInt++) {
						newCubeCoor = cubes.get(cubeInt).getLocation();
							
						if (newCubeCoor != null) {
							requestedVector = new Vector3f((float) (newCubeCoor.getX() - droneX), (float) (newCubeCoor.getY() - droneY), (float) (newCubeCoor.getZ() - droneZ));

							forwardVector = transformVector(toDrone, requestedVector);
							headingVector = new Vector3f(forwardVector.x, 0, forwardVector.z).normalise(null);
							
							headingReq = (float) (Math.atan2(-headingVector.x, -headingVector.z));
							pitchReq = (float) (Math.atan2(forwardVector.y, Vector3f.dot(forwardVector, headingVector)));
								
							if (Math.abs(headingReq) < Math.abs(headingReqMin)) {
								headingReqMin = headingReq;
								pitchReqMin = pitchReq;
							}	
						}
					}
						
					return physics.output((float) (pitchReqMin), (float) (headingReqMin), false, amountOfCubes);
				} else
					return physics.output(1000f, 1000f, false, amountOfCubes);				
			}
		}		
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

	
	/**
	 * Notify the autopilot that the simulation started.
	 * 
	 * @param 	config
	 * 			The configuration of the drone this autopilot represents.
	 * @param 	inputs
	 * 			The current situation of the drone this autopilot steers.
	 * @return	Output commands for the drone, based on the given input parameters.
	 */
    public AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs) {
    		analyser.clearHistory();
		setConfiguration(config);
		physics = new AutopilotPhysics(configuration.getGravity(), configuration.getWingX(), configuration.getTailSize(),
				configuration.getEngineMass(),configuration.getWingMass(), configuration.getTailMass(), configuration.getMaxThrust(),
				configuration.getMaxAOA(), configuration.getWingLiftSlope(), configuration.getHorStabLiftSlope(),
				configuration.getVerStabLiftSlope());
		return analyseInputs(inputs, true);
	}
	
    /**
     * Notify this autopilot of the new situation for the drone it's steering.
     * 
     * @param 	inputs
     * 			The updated situation for the drone.
     * @return	Output commands for steering the drone, based on its most recent situation.
     */
    public AutopilotOutputs timePassed(AutopilotInputs inputs) {
    		return analyseInputs(inputs, false);
    }
    
    /**
     * End the simulation.
     */
    public void simulationEnded() {
    		analyser.clearHistory(); // Currently done both on ending and starting
    }
    
}


