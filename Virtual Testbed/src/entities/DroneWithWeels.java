package entities;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static tools.Tools.addVectors;
import static tools.Tools.scaleVector;
import static tools.Tools.transformVector;

import java.util.ArrayList;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import interfaces.AutopilotOutputs;
import swing_components.ConfigPanel;

public class DroneWithWeels extends Drone {
	
//	private boolean wheelspressed1;
//	private boolean wheelspressed2;
//	private boolean wheelspressed3;

//	private Vector3f wheelposition1;
//	private Vector3f wheelposition2;
//	private Vector3f wheelposition3;
//	
//	
//	
//	
//	private float wheel1brake;
//	private float wheel2brake;
//	private float wheel3brake;
	
	private Wheel frontWheel;
	private Wheel leftWheel;
	private Wheel rightWheel;
	
	//the forces of gravity and the drone tyreslope
	//should neutralize eachother when the drone is at -wheelpos
	//in other words if tyreradius is fully pressed it is equal to drones 
	//gravitational force
	//the dampslope is used for extra damping upon landing
	//but also boosts the leaving a little.
	private float tyreRadius=0.2f;
	//see https://www.quora.com/How-big-is-an-airplane-tire
	private float tyreSlope=.1f;
	private float dampSlope;// dampslope is vrij te kiezen dampende beweging moet sterk zijn
	//zodat tyreradius minimume niet overschreden wordt
	
	private final static Vector3f FRONTWHEELPOSITION = new Vector3f(0,-4.5f,-8.2f);
	private final static Vector3f LEFTWHEELPOSITION = new Vector3f(-4.55f,-4.5f,2f);
	private final static Vector3f RIGHTWHEELPOSITION = new Vector3f(4.55f,-4.5f,2f);
	
	private float fcMax=0.91f;
	
	public DroneWithWeels(float engineMass, float wingMass, float wingX, float tailMass, float tailZ, float wingSlope,
			float horStabSlope, float verStabSlope, float MAXAOA) {
		super(engineMass, wingMass, wingX, tailMass, tailZ, wingSlope, horStabSlope, verStabSlope, MAXAOA);
		// TODO generate wheels etc
	
		frontWheel = new Wheel(FRONTWHEELPOSITION, tyreRadius);
		calculatePressedDistance(frontWheel);

		leftWheel = new Wheel(LEFTWHEELPOSITION, tyreRadius);
		calculatePressedDistance(leftWheel);
		
		rightWheel = new Wheel(RIGHTWHEELPOSITION, tyreRadius);
		//leftWheel.setBrakeForce(3000);
		calculatePressedDistance(rightWheel);

		calculateNeutralizingTyreSlope();
	}
	
	public void calculatePressedDistance(Wheel wheel) {
		Vector3f wheelCenterAbsolute = calculateAbsolutePosition(wheel.getPosition());
		if(wheelCenterAbsolute.y >= wheel.getRadius()) {
			wheel.setPressed(0);
		} else {
			float pressedDistance = wheel.getRadius() - wheelCenterAbsolute.y;
			wheel.setPressed(pressedDistance);
		}
	}
	
	//nominal section is 530 mm
	public void calculateNeutralizingTyreSlope(){
		//calculates the tyreslope if the weight and tyreradius of the drone is known.
		//it should be chosen so that there is no force on the drone when it
		//starts at position (0, -wheelY, 0)
		//System.out.println(totalMass);
		float gravitationForce=totalMass*-GRAVITY;
		tyreSlope=(gravitationForce/(3*0.1f*tyreRadius));
		//i'm not sure about this method
		//but assuming it's a damping motion and we can't go beyond 0 there shoulde be no period
		// omega=0 results in this formula
		dampSlope = 1000f;//((float) (Math.sqrt(4*3*tyreSlope*totalMass)))/3;
		//tyreSlope=2;
	}
	
	@Override
	public void timePassed(float timePassed){

		Matrix4f droneToWorld = getDroneToWorldTransformationMatrix(heading, pitch, roll);
		Matrix4f worldToDrone = getWorldToDroneTransformationMatrix(heading, pitch, roll);
		
		Vector3f relativeVelocity = transformVector(worldToDrone, velocity);
		Vector3f relativeAngularVelocity = transformVector(worldToDrone, angularVelocity);
		
		Vector3f deltaPos = scaleVector(velocity, timePassed);
		this.position = addVectors(deltaPos, position);
		
		Vector3f deltaAngle = scaleVector(relativeAngularVelocity, timePassed);
		Vector3f newForward = addVectors(new Vector3f(0,0,-1),Vector3f.cross(deltaAngle, new Vector3f(0,0,-1), null));
		Vector3f newRight = addVectors(new Vector3f(1,0,0),Vector3f.cross(deltaAngle, new Vector3f(1,0,0), null));
		Vector3f forwardVector = transformVector(droneToWorld, newForward);
		Vector3f headingVector = (new Vector3f(forwardVector.x,0,forwardVector.z)).normalise(null);
		Vector3f rightVector = transformVector(droneToWorld, newRight);
		Vector3f R0 = Vector3f.cross(headingVector, new Vector3f(0,1,0), null);
		Vector3f U0 = Vector3f.cross(R0, forwardVector, null);
		
		heading = (float) atan2(-headingVector.x, -headingVector.z);
		pitch = (float) atan2(forwardVector.y, Vector3f.dot(forwardVector, headingVector));
		roll = (float) atan2(Vector3f.dot(rightVector, U0), Vector3f.dot(rightVector, R0));
		
		Vector3f relativeAirSpeed = relativeVelocity;
		
		Vector3f leftWingAV = new Vector3f(0,(float) sin(leftWingInclination),-(float)cos(leftWingInclination));
		Vector3f leftWingLift = calculateLiftForce(relativeAirSpeed, leftWingAV, new Vector3f(1,0,0), wingSlope);
		
		Vector3f rightWingAV = new Vector3f(0,(float) sin(rightWingInclination),-(float)cos(rightWingInclination));
		Vector3f rightWingLift = calculateLiftForce(relativeAirSpeed, rightWingAV, new Vector3f(1,0,0), wingSlope);
		
		Vector3f horStabAV = new Vector3f(0,(float) sin(horStabInclination),-(float)cos(horStabInclination));
		Vector3f horStabLift = calculateLiftForce(relativeAirSpeed, horStabAV, new Vector3f(1,0,0), horStabSlope);
		
		Vector3f verStabAV = new Vector3f(-(float)sin(verStabInclination), 0, -(float)cos(verStabInclination));
		Vector3f verStabLift = calculateLiftForce(relativeAirSpeed, verStabAV, new Vector3f(0,1,0), verStabSlope);
		
		Vector3f leftWingMoment = Vector3f.cross(new Vector3f(-wingX,0,0), leftWingLift, null);
		Vector3f rightWingMoment = Vector3f.cross(new Vector3f(wingX,0,0), rightWingLift, null);
		Vector3f horStabMoment = Vector3f.cross(new Vector3f(0,0,tailZ), horStabLift, null);
		Vector3f verStabMoment = Vector3f.cross(new Vector3f(0,0,tailZ), verStabLift, null);
		
		Vector3f droneGravity = transformVector(worldToDrone, new Vector3f(0, totalMass*GRAVITY, 0));
		
		
		
		ArrayList<Vector3f> groundForces = calculateGroundLiftForce(timePassed);
		Vector3f groundLiftForce = addVectors(groundForces.get(0), groundForces.get(1),groundForces.get(2));
		Vector3f groundLiftForceRelative = transformVector(worldToDrone, groundLiftForce);
		ArrayList<Vector3f> onGroundBrakefrict=calculateOnGroundBrakeFrictForce(groundForces,timePassed, worldToDrone);
		Vector3f groundBrakeFrictForce = addVectors(onGroundBrakefrict.get(0), onGroundBrakefrict.get(1),onGroundBrakefrict.get(2));
		
		
		Vector3f wheelMomentNormal1 = calculateWheelMoment(frontWheel, groundForces.get(0), droneToWorld);
		Vector3f wheelMomentNormal2 = calculateWheelMoment(leftWheel, groundForces.get(1), droneToWorld);
		Vector3f wheelMomentNormal3 = calculateWheelMoment(rightWheel, groundForces.get(2), droneToWorld);

		Vector3f totalWorldMoment = addVectors(wheelMomentNormal1, wheelMomentNormal2, wheelMomentNormal3);
		Vector3f totalWorldMomentRelative = transformVector(worldToDrone, totalWorldMoment);
		
		Vector3f wheelMomentFrict1 = calculateWheelBottomMoment(frontWheel, onGroundBrakefrict.get(0), worldToDrone);
		Vector3f wheelMomentFrict2 = calculateWheelBottomMoment(leftWheel, onGroundBrakefrict.get(1), worldToDrone);
		Vector3f wheelMomentFrict3 = calculateWheelBottomMoment(rightWheel, onGroundBrakefrict.get(2), worldToDrone);
		
		Vector3f totalFrictMomentRelative = addVectors(wheelMomentFrict1, wheelMomentFrict2, wheelMomentFrict3);
		//ArrayList<Vector3f> onGroundForces=calculateOnGroundForces(totalForce.y,timePassed);
		//Vector3f ongroundliftforce=onGroundForces.get(0);
		//totalMoment.translate(ongroundliftforce.x, ongroundliftforce.y, ongroundliftforce.z);
		//Vector3f ongroundmomentum=onGroundForces.get(1);
		//totalMoment.translate(ongroundmomentum.x,ongroundmomentum.y, ongroundmomentum.z);
	

		Vector3f totalForce = addVectors(leftWingLift, rightWingLift, horStabLift, verStabLift, new Vector3f(0,0,-thrust), droneGravity, groundLiftForceRelative, groundBrakeFrictForce);
		Vector3f totalMoment = addVectors(leftWingMoment, rightWingMoment, horStabMoment, verStabMoment, totalWorldMomentRelative, totalFrictMomentRelative);
		
		Vector3f velocityAcceleration = scaleVector(totalForce, 1/totalMass);
		Vector3f angularAcceleration = new Vector3f(totalMoment.x/inX, totalMoment.y/inY, totalMoment.z/inZ);
	
		velocity = addVectors(transformVector(droneToWorld,scaleVector(velocityAcceleration, timePassed)), velocity);
		
		relativeAngularVelocity = addVectors(relativeAngularVelocity,scaleVector(angularAcceleration, timePassed));
		
		angularVelocity = transformVector(droneToWorld, relativeAngularVelocity);
		
		if(isCrashed()){
			System.out.println("The drone crashed");
			throw new RuntimeException("Drone crashed because it touched the ground");
	
		}
		
	}

	private ArrayList<Vector3f> calculateOnGroundBrakeFrictForce(ArrayList<Vector3f> groundForces, float timePassed, Matrix4f worldToDrone) {
		checkWheelsPressed();
		Vector3f forceByWheel1 = new Vector3f();
		Vector3f forceByWheel2 = new Vector3f();
		Vector3f forceByWheel3 = new Vector3f();
		ArrayList<Vector3f> wheelLiftForces = new ArrayList<>();
	
		//force on every wheel on the ground is  the tireslope force+
		if(frontWheel.isPressed()){
			forceByWheel1 =new Vector3f();// calculateFrictForce(frontWheel,groundForces.get(0),timePassed, worldToDrone);
			forceByWheel1=Vector3f.add(forceByWheel1,calculateBrakeForce(frontWheel,timePassed,worldToDrone),new Vector3f());
		}
		wheelLiftForces.add(forceByWheel1);
		
		if(leftWheel.isPressed()){		
			forceByWheel2 = calculateFrictForce(leftWheel,groundForces.get(1),timePassed, worldToDrone);
			forceByWheel2=Vector3f.add(forceByWheel2,calculateBrakeForce(leftWheel,timePassed,worldToDrone),new Vector3f());
			
		}
		wheelLiftForces.add(forceByWheel2);

		if(rightWheel.isPressed()){
			forceByWheel3 = calculateFrictForce(rightWheel,groundForces.get(2),timePassed, worldToDrone);	
			forceByWheel3=Vector3f.add(forceByWheel3,calculateBrakeForce(rightWheel,timePassed,worldToDrone),new Vector3f());
			
		}
		wheelLiftForces.add(forceByWheel3);


		//System.out.println("the total force"+forceByWheel1);
		//System.out.println("the total force"+forceByWheel2);
		//System.out.println("the total force"+forceByWheel3);
		return wheelLiftForces;
	}

	private Vector3f calculateBrakeForce(Wheel wheel, float timePassed, Matrix4f worldToDrone) {

		Vector3f totalForce;
		Vector3f relativeVelocity = transformVector(worldToDrone, velocity);
		Vector3f relativeAngularVelocity = transformVector(worldToDrone, angularVelocity);
		
		Vector3f rotationVelocity = Vector3f.cross(relativeAngularVelocity, wheel.getPosition(), null);
		//rotationVelocity=(Vector3f) rotationVelocity.scale(-1);
	
		//System.out.println(relativeVelocity);
		Vector3f totalVelocity = addVectors(relativeVelocity, rotationVelocity);
		if(totalVelocity.z<0){
			totalForce=new Vector3f(0.0f, 0.0f, wheel.getBrakeForce());
		}else{
			if(totalVelocity.z>0){
				totalForce=new Vector3f(0.0f, 0.0f, -wheel.getBrakeForce());
			}else{
				totalForce=new Vector3f(0,0,0);
			}
		}
		
		
		
	//	System.out.println("brakeforce"+totalForce);
		return totalForce;
	}

	private Vector3f calculateFrictForce(Wheel wheel, Vector3f force, float timePassed, Matrix4f worldToDrone) {
		// TODO Auto-generated method stub
		float forceValue = force.length();
		//System.out.println("velocity"+velocity);
		Vector3f relativeVelocity = transformVector(worldToDrone, velocity);
		Vector3f relativeAngularVelocity = transformVector(worldToDrone, angularVelocity);
		
		Vector3f rotationVelocity = Vector3f.cross(relativeAngularVelocity, wheel.getPosition(), null);
		//rotationVelocity=(Vector3f) rotationVelocity.scale(-1);
	
		//System.out.println(relativeVelocity);
		Vector3f totalVelocity = addVectors(relativeVelocity, rotationVelocity);
		
		Vector3f totalForce=new Vector3f(((float)(-fcMax*totalVelocity.x*forceValue)), 0.0f, 0.0f);
		
	
		
		return totalForce;
	}

	public ArrayList<Vector3f> calculateGroundLiftForce(float timepassed){
		checkWheelsPressed();
		Vector3f forceByWheel1 = new Vector3f();
		Vector3f forceByWheel2 = new Vector3f();
		Vector3f forceByWheel3 = new Vector3f();
		ArrayList<Vector3f> wheelLiftForces = new ArrayList<>();
	
		//force on every wheel on the ground is  the tireslope force+
		if(frontWheel.isPressed()){
			forceByWheel1 = calculateGroundForce(frontWheel,timepassed);
		}
		wheelLiftForces.add(forceByWheel1);
		
		if(leftWheel.isPressed()){		
			forceByWheel2 = calculateGroundForce(leftWheel,timepassed);
		}
		wheelLiftForces.add(forceByWheel2);

		if(rightWheel.isPressed()){
			forceByWheel3 = calculateGroundForce(rightWheel,timepassed);	
		}
		wheelLiftForces.add(forceByWheel3);

		return wheelLiftForces;
	}

	private Vector3f calculateGroundForce(Wheel wheel, float timePassed){
		float oldPressedDistance = wheel.getPressed();
		calculatePressedDistance(wheel);
		
		float groundForceLength = Math.max(0, wheel.getPressed()*tyreSlope + (dampSlope*((wheel.getPressed()-oldPressedDistance)/timePassed)));
		//System.out.println("Ground force up: " + groundForceLength);
		Vector3f groundForce = new Vector3f(0,groundForceLength,0);
		
		return groundForce;
	}
	
	private Vector3f calculateWheelMoment(Wheel wheel, Vector3f wheelForce, Matrix4f droneToWorld) {
		Vector3f centerToWheelVector = transformVector(droneToWorld, wheel.getPosition());
		return Vector3f.cross(centerToWheelVector, wheelForce, null);
	}

	private Vector3f calculateWheelBottomMoment(Wheel wheel, Vector3f wheelForce, Matrix4f worldToDrone) {
		Vector3f wheelCenterToBottomVector = transformVector(worldToDrone, new Vector3f(0, -wheel.getRadius(), 0));
		Vector3f centerToWheelBottomVector = addVectors(wheel.getPosition(), wheelCenterToBottomVector);
		return Vector3f.cross(centerToWheelBottomVector, wheelForce, null);
	}
	
	//calculates the resistance of the ground when the weels touch the landing space
	//calculated by the drone angels and velocity when touching the ground.
	//additional parameters
	


	private void checkWheelsPressed() {
		frontWheel.setWheelPressed(isWheelPressed(frontWheel));
		leftWheel.setWheelPressed(isWheelPressed(leftWheel));
		rightWheel.setWheelPressed(isWheelPressed(rightWheel));	
	}
	public boolean isCrashed(){
		if(isWheelCrashed(frontWheel)||isWheelCrashed(leftWheel)||isWheelCrashed(rightWheel)){
			return true;
		}
		return super.isCrashed();
	}
	
	private boolean isWheelCrashed(Wheel wheel){
		Vector3f absoluteWheelPosition = calculateAbsolutePosition(wheel.getPosition());
		//System.out.println(absoluteWheelPosition.y);
		if(absoluteWheelPosition.y < 0){
			return true;
		}
		return false;
	}

	private boolean isWheelPressed(Wheel wheel) {
		Vector3f absoluteWheelPosition = calculateAbsolutePosition(wheel.getPosition());
		if(absoluteWheelPosition.y < wheel.getRadius()){
			return true;
		}
		return false;
	}
	
	@Override
	public void setInputs(AutopilotOutputs outputs){
		super.setInputs(outputs);
		
		frontWheel.setBrakeForce(outputs.getFrontBrakeForce());
		leftWheel.setBrakeForce(outputs.getLeftBrakeForce());
		rightWheel.setBrakeForce(outputs.getRightBrakeForce());
	}

	//the plane with wheels should be able to brake
	//forces should be less than RMAX
	
	public void setOtherConfigs(ConfigPanel configPanel){
		float wheelY = frontWheel.getPosition().y;
		float frontZ = frontWheel.getPosition().z;
		float rearZ = leftWheel.getPosition().z;
		float rearX = rightWheel.getPosition().x;
		
		configPanel.setOtherDroneSettings(wheelY, frontZ, rearZ, rearX,
				tyreSlope, dampSlope, tyreRadius, frontWheel.RMAX, fcMax);
	}
}
