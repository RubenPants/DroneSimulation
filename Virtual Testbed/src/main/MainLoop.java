package main;

import interfaces.DroneAutopilot;
import control.AppManager;

public class MainLoop {

	public static void main(String[] args) {

		AppManager.createApp();
		AppManager.setAutopilot(new DroneAutopilot());
		AppManager.loadPath("res/easy.txt");

		while (!AppManager.closeRequested()) {
			AppManager.updateApp();
		}
		AppManager.closeApp();
		System.exit(0);
	}
}
