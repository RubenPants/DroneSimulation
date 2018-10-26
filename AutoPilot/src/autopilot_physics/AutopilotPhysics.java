package autopilot_physics;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;
import org.lwjgl.util.vector.Vector4f;

import interfaces.AutopilotInputs;
import interfaces.AutopilotOutputs;

/**
 * A class of physical models for autopiloting a drone.
 * 
 * @author Team Saffier
 * @version 3.0
 */
public class AutopilotPhysics {

	// *** VARIABLES *** //
	
	private final float gravity;
	private final float wingX;
	private final float tailSize;
	private final float engineMass;
	private final float wingMass;
	private final float tailMass;
	private final float maxThrust;
	private final float maxAOA;
	private final float wingLiftSlope;
	private final float horStabSlope;
	private final float verStabSlope;
	private final float engineZ;

	private float minSSquared;
	private float maxSSquared;
	private float requestedSSquared;
	private float averageWingInclination;
	private float adjustInclination;
	private float thrust;
	private float AOA;
	private float maxRoll = (float) (Math.PI/5); // 36 degrees
	private float maxAdjustInclination = (float)(Math.PI/72); // 2.5 degrees deviation
	private float requestedHeading = 0;
	private float requestedPitch = 0;
	
	private int searchingCubeCounter = 0;
	private int searchingCubePitchDirection = 1;
	private float x = 0;
	private float y = 0;
	private float z = 0;
	private float heading = 0;
	private float pitch = 0;
	private float roll = 0;
	private float previousRoll = 0;
	private float previousTimeElapsed = -0.017f;
	private float timeElapsed = 0;
	private float deltaTimeElapsed = 0.017f;
	private boolean searchingCubeTurnRight = true;
	private Vector3f airSpeed = new Vector3f(0,0,(float)-Math.sqrt(requestedSSquared));


	// *** CONSTRUCTOR *** //
	
	public AutopilotPhysics(float gravity, float wingX, float tailSize, float engineMass, float wingMass, float tailMass,
			float maxThrust, float maxAOA, float wingLiftSlope, float horStabSlope, float verStabSlope) {
		super();
		this.gravity = gravity;
		this.wingX = wingX;
		this.tailSize = tailSize;
		this.engineMass = engineMass;
		this.wingMass = wingMass;
		this.tailMass = tailMass;
		this.maxThrust = maxThrust;
		if (maxAOA > 0.80f)	// The most optimal maxAOA = 0.86rad, but directly there after its force will decrease rapidly. 0.8f is chosen for certainty.
			this.maxAOA = 0.80f;
		else
			this.maxAOA = maxAOA;
		this.wingLiftSlope = wingLiftSlope;
		this.horStabSlope = horStabSlope;
		this.verStabSlope = verStabSlope;
		this.engineZ = calculateEnginePos(tailMass, tailSize, engineMass);
		this.minSSquared = (float) Math.abs((1.2*gravity*getTotalMass())/(maxAOA*Math.cos(maxAOA/2)*wingLiftSlope*Math.cos(roll)*Math.cos(pitch)));
		this.maxSSquared = 2*minSSquared;
		this.requestedSSquared = (float) minSSquared;
	}

	public void getInputsStart(AutopilotInputs inputs){
		// Not used
	}
	
	public void getInputs(AutopilotInputs inputs){
		float currentX = inputs.getX();
		float currentY = inputs.getY();
		float currentZ = inputs.getZ();
		
		heading = inputs.getHeading();
		pitch = inputs.getPitch();
		previousRoll = roll;
		roll = inputs.getRoll();
		timeElapsed = inputs.getElapsedTime();
		deltaTimeElapsed = (timeElapsed - previousTimeElapsed);
		previousTimeElapsed = timeElapsed;
		if (deltaTimeElapsed == 0.0) {
			deltaTimeElapsed = 0.017f;
		}
		airSpeed = Tools.scaleVector(Vector3f.sub(new Vector3f(currentX,currentY,currentZ), new Vector3f(x,y,z), null),1/deltaTimeElapsed);
		
		x = currentX;
		y = currentY;
		z = currentZ;

		minSSquared = (float) Math.abs((1.2*gravity*getTotalMass())/(maxAOA*Math.cos(maxAOA/2)*wingLiftSlope*Math.cos(roll)*Math.cos(pitch)));
		maxSSquared = 2*minSSquared;
				
		System.out.println("_____________________________________________________");
		System.out.println("");
		System.out.println("POSITION: ");
		System.out.println("---X: " + x);
		System.out.println("---Y: " + y);
		System.out.println("---Z: " + z);
		System.out.println("---minSSquared: "+minSSquared);
		System.out.println("---maxSSquared: "+maxSSquared);
		System.out.println("---Time elapsed: " + deltaTimeElapsed);
	}
	
	public AutopilotOutputs output(float reqPitch, float reqHeading, boolean cubeVisible, int amountOfCubesLeft) {

		Matrix4f targetMatrix = getDroneToWorldTransformationMatrix(reqHeading, reqPitch, 0);
		Vector3f targetVector = transformVector(targetMatrix, new Vector3f(0,0,-1));
	
		Matrix4f toDrone = getWorldToDroneTransformationMatrix(heading, pitch, roll);
		Matrix4f toWorld = getDroneToWorldTransformationMatrix(heading, pitch, roll);

		Vector3f worldTargetVector = transformVector(toWorld, targetVector);
		Vector3f headingVector = (new Vector3f(worldTargetVector.x,0,worldTargetVector.z)).normalise(null);
		
		// Map the requestedPitch and -Heading from the drone's point of view to the world's.
		requestedPitch = (float) (atan2(worldTargetVector.y, Vector3f.dot(worldTargetVector, headingVector)));
		requestedHeading = (float) (atan2(-headingVector.x, -headingVector.z));
		
		if (!cubeVisible) {
			searchingCubeCounter++;
			if (searchingCubeCounter > 500) {
				requestedHeading = heading;
				requestedPitch = (float)(searchingCubePitchDirection*Math.PI/18); // PI/18 equals 10 degrees
				
				if (searchingCubeCounter > 800 - Math.random()*200) {
					searchingCubePitchDirection *= -1;
					searchingCubeCounter = 0;
				}
			} else {
				float deltaHeading = requestedHeading - heading;
				if (deltaHeading > Math.PI)
					deltaHeading -= 2*Math.PI;
				else if (deltaHeading < -Math.PI)
					deltaHeading += 2*Math.PI;
				// Help the autopilot to choose a direction
				
				if (Math.abs(deltaHeading) > 17*Math.PI/18) { 	// 17*PI/18 equals 170 degrees
					if (roll > Math.PI/36)		// PI/36 equals 5 degrees
						deltaHeading = (float) (Math.PI);
					else
						deltaHeading = (float) (-Math.PI);
				}
		
				if (deltaHeading > Math.PI/15)
					requestedHeading = (float) (heading + Math.PI/15);
				else if (deltaHeading < -Math.PI/15)
					requestedHeading = (float) (heading - Math.PI/15);
				
				if (requestedHeading > Math.PI)
					requestedHeading -= 2*Math.PI;
				else if (requestedHeading < -Math.PI)
					requestedHeading += 2*Math.PI;
				
				requestedPitch = 0.1f;
			}
		}
		
		// The requestedPitch will have an unreal big values when no cube is visible
		//  In this case a new requestedPitch and -Heading will be assigned, these will be used in the "Searching Cube" section
		if (reqPitch > 500f) {
			requestedPitch = (float) (pitch - Math.PI/36);
			if (requestedPitch < 0.1)	// 0.1 rad equals 6 degrees
				requestedPitch = 0.1f;
			
			if (roll > 0.1)	// 0.1rad equals 6 degree (sort of)
				searchingCubeTurnRight = false;
			else if (roll < -0.1)
				searchingCubeTurnRight = true;
			
			requestedHeading = (float) (heading - Math.PI/15);	// PI/15 will introduce a roll of almost 28.8 degrees (4/5 of maxRoll)
			if (!searchingCubeTurnRight)
				requestedHeading = (float) (heading + Math.PI/15);
			
			if (requestedHeading > Math.PI)
				requestedHeading -= 2*Math.PI;
			else if (requestedHeading < -Math.PI)
				requestedHeading += 2*Math.PI;
		} else { // Make sure that the requestedPitch is reasonable
			if (requestedPitch > Math.PI/6)	// PI/6 equals 30 degrees
				requestedPitch = (float) (Math.PI/6);
			else if (requestedPitch < -Math.PI/9)	// PI/9 equals 20 degrees
				requestedPitch = (float) (-Math.PI/9);
		}

		Vector3f projAirSpeed = transformVector(toDrone, airSpeed);
		float  sSquared = projAirSpeed.lengthSquared();
		
		// sSquared will be zero in the very first frame of the simulation
		if (sSquared == 0.0f)
			sSquared = minSSquared;
		else if (sSquared < minSSquared*2/3) {
			requestedPitch = (float)(pitch - Math.PI/36);
		}
		
		// Update requestedSSquared
		else {
			float speedFactor = (float) (1 - Math.pow((sSquared/maxSSquared), 2));
			if (speedFactor < 0.1f)
				speedFactor = 0.1f;
			else if (speedFactor > 1f)
				speedFactor = 1f;
			
			float speedIndex;
			float pitchInfluence = (float)(Math.abs(pitch - requestedPitch)/(Math.PI/18 * speedFactor));	// PI/18 equals 10 degrees
			float headingInfluence = (float)(Math.abs(heading - requestedHeading)/(Math.PI/18 * speedFactor));	// PI/36 equals 5 degrees 
			
			// The greater the speedIndex [0..1], the lower the speed
			if (pitchInfluence > headingInfluence) 
				speedIndex = pitchInfluence;
			else
				speedIndex = headingInfluence;
			
			if (speedIndex > 1)
				speedIndex = 1;
			
			// If there is just one cube left, then powerboost to this cube
			if (amountOfCubesLeft == 1 && wingLiftSlope > 1) {
				System.out.println("TURBO: ON");
				maxSSquared = 30*minSSquared;
			}
			
			requestedSSquared = (float) (minSSquared + (maxSSquared - minSSquared) * Math.pow(1 - speedIndex, 2));
		}
		
		float gY = getTotalMass() * gravity;
		Vector3f projGravity = transformVector(toDrone, new Vector3f(0,-gY,0));
				
		if (reqPitch > 500f) {
			System.out.println("");
			System.out.println("DRONE PHYSICS (Searching Cube):");

			searchingCubeCounter++;
			if (searchingCubeCounter > 500) {
				requestedHeading = heading;
				requestedPitch = (float)(searchingCubePitchDirection*Math.PI/18); // PI/18 equals 10 degrees
			}
			
			averageWingInclination = findInclinationZeroForceDroneY(wingLiftSlope, sSquared, projGravity, requestedPitch);

			if (searchingCubeCounter > 800 - Math.random()*200) {
				searchingCubePitchDirection *= -1;
				searchingCubeCounter = 0;
			}
		} else {
			System.out.println("");
			System.out.println("DRONE PHYSICS (Default):");
			
			if (cubeVisible) searchingCubeCounter = 0;
			
			averageWingInclination = findInclinationZeroForceDroneY(wingLiftSlope, sSquared, projGravity, requestedPitch);
		}

		// TODO: Make this better
		// Check if the new averageWingInclination exceeds the maxAOA --> How to (properly) check what the AOA is at each moment?
		Vector3f axis = new Vector3f(1,0,0);
		Vector3f attackVector = new Vector3f(0,(float)sin(averageWingInclination+maxAdjustInclination),-(float)cos(averageWingInclination+maxAdjustInclination));
		Vector3f normal = Vector3f.cross(axis, attackVector, null);
		AOA = (float) -Math.atan2(Vector3f.dot(projAirSpeed, normal), Vector3f.dot(projAirSpeed, attackVector));
		while (Math.abs(AOA) >= Math.abs(maxAOA)*0.99f && Math.abs(averageWingInclination) > 0.01) {
			if (AOA > 0)
				averageWingInclination -= 0.01f;  // 0.01rad equals 0.57 degrees
			else
				averageWingInclination += 0.01f;
			
			if (averageWingInclination > maxAOA)
				averageWingInclination = -maxAOA;
			else if (averageWingInclination < -maxAOA)
				averageWingInclination = maxAOA;
			
			attackVector = new Vector3f(0,(float)sin(averageWingInclination),-(float)cos(averageWingInclination));
			AOA = (float) -Math.atan2(Vector3f.dot(projAirSpeed, normal), Vector3f.dot(projAirSpeed, attackVector));
		}
		
		System.out.println("---pitch: " + Math.toDegrees(pitch));
		System.out.println("---requestedPitch: " + Math.toDegrees(requestedPitch));
		System.out.println("---reqPitch: " + Math.toDegrees(reqPitch));
		System.out.println("---heading: " + Math.toDegrees(heading));
		System.out.println("---requestedHeading: " + Math.toDegrees(requestedHeading));
		System.out.println("---reqHeading: " + Math.toDegrees(reqHeading));
		
		thrust = findThrustZeroForceDroneZ(averageWingInclination, wingLiftSlope, sSquared, projGravity);
		
		adjustInclination = adjustInclinationToMatchHeading(averageWingInclination, requestedHeading, wingLiftSlope, sSquared);
		
		float[] leftAndRightInclination;
		leftAndRightInclination = getLeftAndRightAdjIncl(averageWingInclination, adjustInclination, wingLiftSlope, sSquared);		
		
		System.out.println("---roll: "+ Math.toDegrees(roll));
		System.out.println("---inclination: " + Math.toDegrees(averageWingInclination));
		System.out.println("---adjustInclination: " + Math.toDegrees(adjustInclination));
		System.out.println("---leftIncl: "+ Math.toDegrees(leftAndRightInclination[0]));
		System.out.println("---rightIncl: "+ Math.toDegrees(leftAndRightInclination[1]));
		System.out.println("---sSquared: " + sSquared);
		System.out.println("---requestedSSquared: " + requestedSSquared);
		System.out.println("---thrust: " + thrust);
		if (thrust == maxThrust)
			System.out.println("------maxThrust reached");
				
		return new AutopilotOutputs() {
			
			@Override
			public float getRightWingInclination() {
				return leftAndRightInclination[1];
			}
			
			@Override
			public float getLeftWingInclination() {
				return leftAndRightInclination[0];
			}
			
			@Override
			public float getHorStabInclination() {
				return 0;
			}
			
			@Override
			public float getVerStabInclination() {
				return 0;
			}
			
			@Override
			public float getThrust() {
				return thrust;
			}
		};
	}
	
	
	// *** HELPER METHODS *** //
	
	float[] getLeftAndRightAdjIncl(float averageInclination, float adjustInclination, float liftSlopeConstant, float sSquared) {
		float EPSILON = 0.0000001f;
		float leftIncl  = (averageInclination - adjustInclination);
		float rightIncl = (averageInclination + adjustInclination);
		float initForceWings = 2*wingLift(averageInclination, liftSlopeConstant, sSquared);
		
		float maxAdjIncl = Math.abs(maxAOA - Math.abs(averageInclination));
		if (maxAdjIncl > 0.25f)
			maxAdjIncl = 0.25f;
		
		float a = -maxAdjIncl;
		float fa = wingLift(leftIncl + a, liftSlopeConstant, sSquared) + wingLift(rightIncl + a, liftSlopeConstant, sSquared) - initForceWings;
		float b = maxAdjIncl;
		float fb = wingLift(leftIncl + b, liftSlopeConstant, sSquared) + wingLift(rightIncl + b, liftSlopeConstant, sSquared) - initForceWings;
		
		if (fa*fb > 0) {
			return new float[] {leftIncl, rightIncl};
		}
		
		while((b-a) > EPSILON){
			if((wingLift(leftIncl + (b+a)/2, liftSlopeConstant, sSquared) + wingLift(rightIncl + (b+a)/2, liftSlopeConstant, sSquared) - initForceWings) > 0)
				b = (b+a)/2;
			else
				a = (b+a)/2;
		}
		
		return new float[] {leftIncl + (b+a)/2, rightIncl + (b+a)/2};
	}
	
	private float adjustInclinationToMatchHeading(float averageInclination, float requestedHeading, float liftSlopeConstant, float sSquared) {		
		float angularVelocity = (roll - previousRoll) / deltaTimeElapsed;
		float requestedRoll;
		float requestedAngularVelocity;
		float adjustInclination;
		
		// Decide which is the fastest way to go to the requested heading
		float deltaHeading = (requestedHeading - heading);
		if (deltaHeading > Math.PI)
			deltaHeading -= 2*Math.PI;
		else if (deltaHeading < -Math.PI)
			deltaHeading += 2*Math.PI;
		
		float deltaHeadingFactor = (float) (deltaHeading/(Math.PI/12));
		if (deltaHeadingFactor > 1)
			deltaHeadingFactor = 1;
		else if (deltaHeadingFactor < -1)
			deltaHeadingFactor = -1;
		
		float speedFactor = (sSquared-minSSquared)/(maxSSquared-minSSquared);
		if (speedFactor < 0)
			speedFactor = 0;
		else if (speedFactor > 1)
			speedFactor = 1;

		// For a liftSlope of 0.25 and lower the roll will be between [-36..36] (expressed in degrees)
		// For a liftSlope of 1 and higher the roll will be between [-12..12] (expressed in degrees)
		if (liftSlopeConstant < 0.25)
			requestedRoll = (float) maxRoll*deltaHeadingFactor;
		else if (liftSlopeConstant > 0.75)
			requestedRoll = (float) maxRoll*deltaHeadingFactor/3;
		else
			requestedRoll = (float) maxRoll*deltaHeadingFactor/(4*liftSlopeConstant);

		if (requestedRoll > maxRoll)
			requestedRoll = maxRoll;
		else if (requestedRoll < -maxRoll)
			requestedRoll = maxRoll;		
		
		requestedAngularVelocity = (float) ((requestedRoll-roll) * (1+speedFactor) * (25*verStabSlope));
		if (liftSlopeConstant > 0.6)
			requestedAngularVelocity /= 4;
		else if (liftSlopeConstant > 0.2)
			requestedAngularVelocity /= 2;
		
		
		if (requestedAngularVelocity > 0.5)
			requestedAngularVelocity = (float)(0.5);
		else if (requestedAngularVelocity < -0.5)
			requestedAngularVelocity = (float)(-0.5);
		
		System.out.println("---requestedAngularVelocity: "+requestedAngularVelocity);
		System.out.println("---requestedRoll: " + Math.toDegrees(requestedRoll));

		// Stabilize towards the requestedAngularVelocity
		adjustInclination = findInclinationAngularVelocity(angularVelocity, requestedAngularVelocity, averageInclination, sSquared, (requestedHeading - heading));
			
		return adjustInclination;
	}
	
	private float findInclinationAngularVelocity(float angularVelocity, float requestedAngularVelocity, float averageInclination, float sSquared, float deltaHeading) {
		float EPSILON = 0.0000001f;
		
		float a = -(maxAOA-Math.abs(averageInclination));
		float Fa = forceEquationAdjustInclination(averageInclination, a, sSquared, angularVelocity, requestedAngularVelocity);
		float b = (maxAOA-Math.abs(averageInclination));
		float Fb = forceEquationAdjustInclination(averageInclination, b, sSquared, angularVelocity, requestedAngularVelocity);
		
		if (Fa*Fb > 0) {
			if (requestedAngularVelocity > angularVelocity)
				return maxAdjustInclination;
			else
				return -maxAdjustInclination;
		}
		
		while((b-a) > EPSILON){
			if(forceEquationAdjustInclination(averageInclination, (b+a)/2, sSquared, angularVelocity, requestedAngularVelocity) > 0)
				b = (b+a)/2;
			else
				a = (b+a)/2;
		}

		float result = (b+a)/2;
			
		if (result > maxAdjustInclination)
			return maxAdjustInclination;	
		else if (result < -maxAdjustInclination)
			return -maxAdjustInclination;
		else
			return result;
	}
	
	private float forceEquationAdjustInclination(float averageInclination, float adjustInclination, float sSquared, float angularVelocity, float requestedAngularVelocity) {
		return (float) wingLift(averageInclination + adjustInclination, wingLiftSlope, sSquared) -  // The minus sign is because the direction of the forces are opposite
				wingLift(averageInclination - adjustInclination, wingLiftSlope, sSquared) -
				(requestedAngularVelocity - angularVelocity)*getWingMass()*getWingX()/deltaTimeElapsed;
	}
	
	private float findInclinationZeroForceY(float thrust, float liftSlopeConstant, float sSquared, Vector3f projGravity){
		return findInclinationZeroForceDroneY(liftSlopeConstant, sSquared, projGravity, 0.1f);	// 0.05rad equals 6 degrees
	}
	
	private float findInclinationZeroForceDroneY(float liftSlopeConstant, float sSquared, Vector3f projGravity, float requestedPitch){
		float EPSILON = 0.0000001f;
		float maxAOAFactor = (float) Math.pow(Math.abs(1 - AOA/maxAOA), 0.5f);
		
		float elevationFactor;
		if (pitch < 0 && requestedPitch > 0)
			elevationFactor = (float)((requestedPitch - pitch)/(Math.PI/72)*maxAOAFactor);
		else 
			elevationFactor = (float)((requestedPitch - pitch)/(Math.PI/36)*maxAOAFactor);
		
		if (elevationFactor <= -1)
			elevationFactor = -1;
		else if (elevationFactor > 0)
			elevationFactor *= 1.1;
		if (elevationFactor >= 1)
			elevationFactor = 1;
		
		System.out.println("---elevationFactor: "+elevationFactor);
		
		float a = 0;
		float fa = totalForceDroneY(a, liftSlopeConstant, sSquared);
		float b = maxAOA;
		float fb = totalForceDroneY(b, liftSlopeConstant, sSquared);
			
		if(fa*fb > 0){
			a = -maxAOA;
			fa = totalForceDroneY(a, liftSlopeConstant, sSquared);
			b = 0;
			fb = totalForceDroneY(b, liftSlopeConstant, sSquared);
			
			// The plane is flying upside down --> Free-fall if elevationFactor == 1
			if (elevationFactor == -1) {
				System.out.println("ERROR: FREE-FALL");
				return 0;
			} else if (fa*fb > 0) {		// i.e. the drone's velocity is to low
				System.out.println("ERROR: NO ZERO FOUND! (inclination)"); 
				if (requestedPitch == 0) {	// In this case, the method is in its second iteration, and still failed
					System.out.println("ERROR: FREE-FALL");
					return 0;	
				}
				else
					return findInclinationZeroForceDroneY(liftSlopeConstant, sSquared, projGravity, 0);
			}
		}
		
		while((b-a) > EPSILON){
			if(totalForceDroneY((b+a)/2, liftSlopeConstant, sSquared) > 0)
				b = (b+a)/2;
			else
				a = (b+a)/2;
		}
		
		return adjustInclinationPitch((a+b)/2, elevationFactor, liftSlopeConstant, sSquared);
	}
	
	private float adjustInclinationPitch(float defaultInclination, float elevationFactor, float liftSlopeConstant, float sSquared) {
		float EPSILON = 0.0000001f;

		float defaultLift = wingLift(defaultInclination, liftSlopeConstant, sSquared);
		float requestedLift;
		if (liftSlopeConstant < 0.2) {
			requestedLift = (float) (defaultLift + getTotalMass()*gravity*elevationFactor*(2-Math.cos(roll))/5);
		} else if (liftSlopeConstant < 0.5) {
			requestedLift = (float) (defaultLift + getTotalMass()*gravity*elevationFactor*(2-Math.cos(roll))/6);
		} else {
			requestedLift = (float) (defaultLift + getTotalMass()*gravity*elevationFactor*(2-Math.cos(roll))/8);
		}

		float a;
		float b;
		if (elevationFactor > 0) {
			a = defaultInclination;
			b = maxAOA;
		} else {
			a = -maxAOA;
			b = defaultInclination;
		}
		
		float Fa = wingLift(a, liftSlopeConstant, sSquared) - requestedLift;
		float Fb = wingLift(b, liftSlopeConstant, sSquared) - requestedLift;
		if(Fa*Fb > 0){ // The target can't be reached
			return maxAOA;
		}
		
		while((b-a) > EPSILON){
			if(wingLift((b+a)/2, liftSlopeConstant, sSquared) - requestedLift > 0)
				b = (b+a)/2;
			else 
				a = (b+a)/2;
		}
		
		return (b+a)/2;
	}

	private float findThrustZeroForceDroneZ(float inclinationWings, float liftSlopeConstant, float sSquared, Vector3f projGravity){
		float thrust = (float) (2*inclinationWings*Math.sin(inclinationWings)*liftSlopeConstant*sSquared + projGravity.z);
		float requestedAcceleration;
		float deltaThrust;
		
		if (deltaTimeElapsed == 0)
			requestedAcceleration = 0;
		else
			requestedAcceleration = (requestedSSquared - sSquared)*3;
		// The factor 3 is there because the requestedAcceleration will be reached in 20 frames assuming 60 FPS (so in 0.33 seconds)
	
		deltaThrust = requestedAcceleration * getTotalMass();
		thrust += deltaThrust;			
			
		if (thrust > maxThrust)
			return maxThrust;
		else if (thrust < 0)
			return 0;
		else
			return thrust;
	}
	
	private float wingLift(float inclination, float liftSlopeConstant, float sSquared) {
		return (float) (inclination*Math.cos(inclination)*liftSlopeConstant*sSquared);
	}
	
	private float totalForceDroneY(float inclination, float liftSlopeConstant, float sSquared) {
		Matrix4f toDrone = getWorldToDroneTransformationMatrix(heading, pitch, roll);
		
		float wingLiftY = (float) 2*wingLift(inclination, liftSlopeConstant, sSquared);
		
		Vector3f gravityVector = transformVector(toDrone, new Vector3f(0, -getTotalMass()*gravity, 0));
		float gravityY = (gravityVector.y);
		
		return (wingLiftY + gravityY);
	}
	
	private float getTotalMass(){
		return (engineMass + 2*wingMass + tailMass);
	}
	
	private float getWingMass() {
		return wingMass;
	}
	
	private float getWingX() {
		return wingX;
	}
	
	private float calculateEnginePos(float tailMass, float tailSize, float engineMass) {
		return -(tailMass * tailSize) / engineMass;
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
	
	private Vector3f transformVector(Matrix4f matrix, Vector3f vector) {
		Vector4f vector4 = new Vector4f(vector.x, vector.y, vector.z, 1);
		Matrix4f.transform(matrix, vector4, vector4);
		
		return new Vector3f(vector4.x, vector4.y, vector4.z);
	}
	
	public static void printVector(String name, Vector3f vector) {
		System.out.println(name + ": [" + vector.x + ", " + vector.y + ", " + vector.z + "]");
	}
	
	
	// *** UNUSED HELPER METHODS *** //
	
	private float totalForceX(float inclination, float thrust, float liftSlopeConstant, float sSquared) {
		Matrix4f toWorld = getDroneToWorldTransformationMatrix(heading, pitch, roll);
		
		Vector3f thrustUnitVector = transformVector(toWorld, new Vector3f(0,0,-1));
		float thrustX = (float) (thrust * thrustUnitVector.x);
		
		Vector3f wingNormalUnitVector = transformVector(toWorld, new Vector3f(0, (float) Math.cos(inclination), (float) Math.sin(inclination)));
		float wingLiftX = (float) ((2*inclination*liftSlopeConstant*sSquared) * wingNormalUnitVector.x);
		
		return (thrustX + wingLiftX);
	}
	
	private float totalForceY(float inclination, float thrust, float liftSlopeConstant, float sSquared) {
		Matrix4f toWorld = getDroneToWorldTransformationMatrix(heading, pitch, roll);
		
		Vector3f thrustUnitVector = transformVector(toWorld, new Vector3f(0,0,-1));
		float thrustY = (float) (thrust * thrustUnitVector.y);
		
		Vector3f wingNormalUnitVector = transformVector(toWorld, new Vector3f(0, (float) Math.cos(inclination), (float) Math.sin(inclination)));
		float wingLiftY = (float) ((2*inclination*liftSlopeConstant*sSquared) * wingNormalUnitVector.y);
		
		float gravityY = (-getTotalMass() * gravity);		
		
		return (thrustY + wingLiftY + gravityY);
	}
	
	private float totalForceZ(float inclination, float thrust, float liftSlopeConstant, float sSquared) {
		Matrix4f toWorld = getDroneToWorldTransformationMatrix(heading, pitch, roll);
		
		Vector3f thrustUnitVector = transformVector(toWorld, new Vector3f(0,0,-1));
		float thrustZ = (float) (thrust * thrustUnitVector.z);
		
		Vector3f wingNormalUnitVector = transformVector(toWorld, new Vector3f(0, (float) Math.cos(inclination), (float) Math.sin(inclination)));
		float wingLiftZ = (float) ((2*inclination*liftSlopeConstant*sSquared) * wingNormalUnitVector.z);
		
		return (thrustZ + wingLiftZ);
	}
	
	private float totalForceDroneX() {
		Matrix4f toDrone = getWorldToDroneTransformationMatrix(heading, pitch, roll);
		
		Vector3f gravityVector = transformVector(toDrone, new Vector3f(0, -getTotalMass()*gravity, 0));
		
		return gravityVector.x;
	}
	
	private float totalForceDroneZ(float inclinationWings, float thrust, float liftSlopeConstant, float sSquared) {
		Matrix4f toDrone = getWorldToDroneTransformationMatrix(heading, pitch, roll);
		
		float wingLiftZ = (float) (2*inclinationWings*liftSlopeConstant*sSquared*Math.sin(inclinationWings));
		
		Vector3f gravityVector = transformVector(toDrone, new Vector3f(0, -getTotalMass()*gravity, 0));
		float gravityZ = gravityVector.z;
		
		return (-thrust + wingLiftZ + gravityZ);
	}
	
}



