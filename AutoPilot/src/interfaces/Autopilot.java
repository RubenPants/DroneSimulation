package interfaces;

/**
 * A type of autopilots for steering a drone. Autopilots translate a drone's configuration and
 *  constantly updated situation to output commands, such that that very drone can reach its 
 *  desired destination.
 * 
 * @author 	Team Saffier
 * @version 	1.0
 */
public interface Autopilot {
	
	/**
	 * Notify the autopilot that the simulation started.
	 * 
	 * @param 	config
	 * 			The configuration of the drone this autopilot represents.
	 * @param 	inputs
	 * 			The current situation of the drone this autopilot steers.
	 * @return	Output commands for the drone, based on the given input parameters.
	 */
    AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs);
    
    /**
     * Notify this autopilot of the new situation for the drone it's steering.
     * 
     * @param 	inputs
     * 			The updated situation for the drone.
     * @return	Output commands for steering the drone, based on its most recent situation.
     */
    AutopilotOutputs timePassed(AutopilotInputs inputs);
    
    /**
     * End the simulation.
     */
    void simulationEnded();
    
}