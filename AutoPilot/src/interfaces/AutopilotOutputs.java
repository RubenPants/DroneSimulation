package interfaces;

/**
 * An interface for autopilot output objects having a thrust, left - and right wing
 * 	inclination, and horizontal - and vertical stabiliser inclination.
 * 
 * @author	Team Saffier
 * @version	1.0
 */
public interface AutopilotOutputs {
	
	/**
	 * Return the thrust value for this output object.
	 */
    float getThrust();
    
    /**
	 * Return the left wing inclination value for this output object.
	 */
    float getLeftWingInclination();
    
    /**
	 * Return the right wing inclination value for this output object.
	 */
    float getRightWingInclination();
    
    /**
	 * Return the horizontal stabiliser inclination value for this output object.
	 */
    float getHorStabInclination();
    
    /**
	 * Return the vertical stabiliser inclination value for this output object.
	 */
    float getVerStabInclination();
    
}