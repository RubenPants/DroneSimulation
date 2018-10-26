package interfaces;

public class AutopilotFactory {

	public static Autopilot createAutopilot() {
		
		return new DroneAutopilot();
	}
}