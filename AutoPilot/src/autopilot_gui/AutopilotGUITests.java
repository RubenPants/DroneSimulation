package autopilot_gui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JCheckBox;

/**
 * A class for testing autopilot GUIs.
 * 
 * @author	Team Saffier
 * @version	1.0
 */
public class AutopilotGUITests {
	
	/**
	 * Variable registering the GUI used by this test case.
	 */
	private static AutopilotGUI gui;

	/**
	 * Test out the progress GUI.
	 */
	public static void main(String[] args) {
		gui = new AutopilotGUI();
		gui.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				JCheckBox box = ((JCheckBox)e.getSource());
				System.out.println(box.isSelected());
			}
		});
	}

}