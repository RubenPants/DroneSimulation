package entities;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static tools.Tools.addVectors;
import static tools.Tools.scaleVector;
import static tools.Tools.transformVector;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import interfaces.AutopilotOutputs;
import worldSimulation.DroneStartSettings;

public class Drone {
	
	private float GRAVITY = -9.81f;
	private final float MAX_AOA;
	
	private TexturedModel model;
	
	private final float totalMass;
	private final float engineZ;
	private final float wingX;
	private final float tailZ;
	
	private final float wingSlope;
	private final float horStabSlope;
	private final float verStabSlope;
	
	private final float inX; //traagheidsmoment in kg*m2 as van de vleugels / bij pitch draaiing
	private final float inZ; //traagheidsmoment bij roll draaiing / om de as van de romp
	private final float inY; //traagheidsmoment bij heading draaiing / om de as loodrecht door vliegtuig (is gwn de som van de andere 2)
	
	private Vector3f position = new Vector3f(0,0,0);	//world coo
	private Vector3f velocity = new Vector3f(0,0,0);	//world coo
	private Vector3f angularVelocity = new Vector3f(0,0,0); //world coo
	
	private float heading = 0; //links rechts gedraaid in radialen
	private float pitch = 0; //voor onder boven gedraaid in radialen
	private float roll = 0;// (float)Math.PI;	//rond eigen as links rechts gedraaid in radialen
	
	private float thrust;
	private float leftWingInclination;
	private float rightWingInclination;
	private float horStabInclination = 0;
	private float verStabInclination = 0;
	
	public Drone(float engineMass, float wingMass, float wingX, float tailMass, float tailZ, float wingSlope, float horStabSlope, float verStabSlope, float MAXAOA){
		this.totalMass = engineMass+2*wingMass+tailMass;
		this.wingX = wingX;
		this.tailZ = tailZ;
		this.engineZ = -(tailMass*tailZ)/engineMass;
		this.inZ = 2*wingMass*wingX*wingX;
		this.inX = engineMass*engineZ*engineZ + tailMass*tailZ*tailZ;
		this.inY = this.inZ + this.inX;
		this.wingSlope = wingSlope;
		this.horStabSlope = horStabSlope;
		this.verStabSlope = verStabSlope;
		this.velocity = new Vector3f(0,0,(float) -Math.sqrt(Math.abs(1.2*GRAVITY*totalMass)/(MAXAOA*Math.cos(MAXAOA/2)*wingSlope)));
		this.MAX_AOA = MAXAOA;
	}
	
	public void setInputs(AutopilotOutputs outputs){
		this.thrust = outputs.getThrust();
		this.leftWingInclination = (float) outputs.getLeftWingInclination();
		this.rightWingInclination = (float) outputs.getRightWingInclination();
		this.horStabInclination =(float) outputs.getHorStabInclination();
		this.verStabInclination = (float) outputs.getVerStabInclination();
	}
	
	public void setGravity(float gravity) {
		this.GRAVITY = -gravity;
	}
	
	public void reset(DroneStartSettings settings){
		this.position = settings.getPosition();
		this.heading = settings.getHeading();
		this.pitch = settings.getPitch();
		this.roll = settings.getRoll();

		this.angularVelocity = settings.getAngularVelocity();
		Vector3f newVel = new Vector3f(0,0,(float) -Math.sqrt(Math.abs(1.2*GRAVITY*totalMass)/(MAX_AOA*Math.cos(MAX_AOA/2)*wingSlope)));
		this.velocity = transformVector(getDroneToWorldTransformationMatrix(heading, pitch, roll), newVel);
	}
	
	public Vector3f getPosition() {
		return position;
	}
	
	public float getHeading() {
		return heading;
	}
	
	public float getPitch() {
		return pitch;
	}
	
	public float getRoll() {
		return roll;
	}
	
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
		
		Vector3f totalForce = addVectors(leftWingLift, rightWingLift, horStabLift, verStabLift, new Vector3f(0,0,-thrust), droneGravity);
		Vector3f totalMoment = addVectors(leftWingMoment, rightWingMoment, horStabMoment, verStabMoment);
				
		Vector3f velocityAcceleration = scaleVector(totalForce, 1/totalMass);
		Vector3f angularAcceleration = new Vector3f(totalMoment.x/inX, totalMoment.y/inY, totalMoment.z/inZ);
		
		velocity = addVectors(transformVector(droneToWorld,scaleVector(velocityAcceleration, timePassed)), velocity);
		
		relativeAngularVelocity = addVectors(relativeAngularVelocity,scaleVector(angularAcceleration, timePassed));
		
		angularVelocity = transformVector(droneToWorld, relativeAngularVelocity);
	}
	
	private Vector3f calculateLiftForce(Vector3f airSpeed, Vector3f attackVector, Vector3f axis, float slope) {
		Vector3f projAirSpeed = new Vector3f((1-axis.x)*airSpeed.x, (1-axis.y)*airSpeed.y, (1-axis.z)*airSpeed.z);
		Vector3f normal = Vector3f.cross(axis, attackVector, null);
		float AOA = (float) -Math.atan2(Vector3f.dot(projAirSpeed, normal), Vector3f.dot(projAirSpeed, attackVector));
		float factor = slope*AOA*projAirSpeed.lengthSquared();
		if(Math.abs(AOA) > this.MAX_AOA){
//			TODO: max AOA disabled
//			System.out.println("MAX AOA EXCEEDED");
//			throw new RuntimeException("MAXAOA WAS EXCEEDED. Allowed AOA is " + this.MAX_AOA + ". Calculated AOA is " + AOA +".");
		}
		return scaleVector(normal, factor);
	}
	
	private Matrix4f getWorldToDroneTransformationMatrix(float heading, float pitch, float roll) {
		return Matrix4f.invert(getDroneToWorldTransformationMatrix(heading, pitch, roll), null);
	}

	private Matrix4f getDroneToWorldTransformationMatrix(float heading, float pitch, float roll) {
		Matrix4f matrix = new Matrix4f();
		matrix.setIdentity();
		Matrix4f.rotate(heading, new Vector3f(0, 1, 0), matrix, matrix);
		Matrix4f.rotate(pitch, new Vector3f(1, 0, 0), matrix, matrix);
		Matrix4f.rotate(roll, new Vector3f(0, 0, 1), matrix, matrix);
		return matrix;
	}
	public void addModel(TexturedModel model) {
		this.model = model;
	}
	
	public Entity getEntity() {
		/*
		 * scalingFactor = 0.05 --> wingX = 0.25
		 * ...
		 * scalingFactor = 1    --> wingX = 5
		 * ...
		 * scalingFactor = 2    --> wingX = 10
		 * 
		 * --> scalingFactor = wingX/5;
		 */
		float scalingFactor = wingX/5;
		
		if (scalingFactor > 2) // Give a upper boundary for the size of the drone, otherwise the drone would be to large for the screen!
			scalingFactor = 2f;
		else if (scalingFactor < 0.05f) // Give a lower boundary for the size of the drone, otherwise it wouldn't be visible!
			scalingFactor = 0.05f;
		
		return new Entity(model,position,heading,pitch,roll,scalingFactor);
	}
	
	public Vector3f getVelocity() {
		return velocity;
	}
	
	public Vector3f getRelativeAngularVelocity() {
		Matrix4f worldToDrone = getWorldToDroneTransformationMatrix(heading, pitch, roll);
		return transformVector(worldToDrone, angularVelocity);
	}
	
	public int getTextureID(){
		return model.getTexture();
	}
	
	public void increaseRotation(float dheading, float dpitch, float droll) {
		this.heading += dheading;
		this.pitch += dpitch;
		this.roll += droll;
	}
}
