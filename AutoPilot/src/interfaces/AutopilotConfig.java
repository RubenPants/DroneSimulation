package interfaces;

/**
 * An interface for autopilot configurations, having a gravity, distance between the drone's center of gravity and
 *  the point where the wings' mass and lift are located, tail size, engine mass, wing mass, tail mass, maximum 
 *  thrust, maximum angle of attack, wing lift slope, horizontal - and vertical stabiliser lift slope, horizontal - 
 *  and vertical angle of view, and number of columns and rows.
 * 
 * @author	Team Saffier
 * @version	1.0
 */
public interface AutopilotConfig {
	
	/**
	 * Return the  world's gravitational constant (in N/kg).
	 */
    float getGravity();
    
    /**
     * Return the distance between the drone's center of gravity and the point 
     * 	where the wings' mass and lift are located.
     */
    float getWingX();
    
    /**
	 * Return the distance between the drone's center of gravity and the point 
	 * 	where the tail mass and the lift generated by the horizontal and vertical stabilizers
	 *  is located.
	 */
    float getTailSize();
    
    /**
	 * Return the mass of the engine. The engine is located in front of the drone's center of gravity.
	 */
    float getEngineMass();
    
    /**
	 * Return the mass of the left wing. Equals the mass of the right wing. 
	 * 	Modeled as being located in a single point.
	 */
    float getWingMass();
    
    /**
	 * Return mass of the tail. Modeled as being located in a single point.
	 */
    float getTailMass();
    
    /**
	 * Return the maximum forward engine thrust. (Minimum thrust is zero.)
	 */
    float getMaxThrust();
    
    /**
	 * Return the maximum magnitude of the angle of attack of all four airfoils. 
	 *  If during a simulation an airfoil's angle of attack exceeds this value, 
	 *  the simulator may report an error and abort the simulation.
	 */
    float getMaxAOA();
    
    /**
	 * Return the liftSlope value for computing the lift generated by a wing..
	 */
    float getWingLiftSlope();
    
    /**
	 * Return the liftSlope value for the horizontal stabilizer.
	 */
    float getHorStabLiftSlope();
    
    /**
	 * Return the liftSlope value for the vertical stabilizer.
	 */
    float getVerStabLiftSlope();
    
    /**
	 * Return the horizontal angle of view of the camera.
	 */
    float getHorizontalAngleOfView();
    
    /**
	 * Return the vertical angle of view of the camera.
	 */
    float getVerticalAngleOfView();
    
    /**
	 * Return the number of columns of pixels in the camera image.
	 */
    int getNbColumns();
    
    /**
	 * Return the number of rows of pixels in the camera image.
	 */
    int getNbRows();
    
}