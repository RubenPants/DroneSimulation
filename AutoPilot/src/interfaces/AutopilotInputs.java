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
public interface AutopilotInputs {
	
	/**
	 * Returns the camera image, top row first. Within a row, leftmost pixel first.
	 */
    byte[] getImage();
    
    /**
	 * Returns the X coordinate of the drone's center of gravity in world coordinates.
	 */
    float getX();
    
    /**
	 * Returns the Y coordinate of the drone's center of gravity in world coordinates.
	 */
    float getY();
    
    /**
	 * Returns the Z coordinate of the drone's center of gravity in world coordinates.
	 */
    float getZ();
    
    /**
	 * Returns atan2(H . (-1, 0, 0), H . (0, 0, -1)), where H is the drone's heading vector 
	 *  (which we define as the drone's forward vector ((0, 0, -1) in drone coordinates) 
	 *  projected onto the world XZ plane.
	 */
    float getHeading();
    
    /**
	 * Returns atan2(F . (0, 1, 0), F . H), where F is the drone's forward vector and H is 
	 *  the drone's heading vector.
	 */
    float getPitch();
    
    /**
	 * Returns atan2(R . (0, 1, 0), R . R0), where R is the drone's right direction ((1, 0, 0) 
	 *  in drone coordinates) and R0 = H x (0, 1, 0).
	 */
    float getRoll();
    
    /**
	 * Returns the amount of simulated time elapsed since the start of the simulation. Need not 
	 *  bear any relationship to real time (other than increasing).
	 */
    float getElapsedTime();
    
}