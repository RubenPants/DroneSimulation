package autopilot_gui;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.event.ActionListener;

import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;

/**
 * A class of autopilot interfaces for letting a use interact with an autopilot.
 * 
 * @author	Team Saffier
 * @version	1.0
 */
public class AutopilotGUI {
	
	/**
	 * Initialise this new GUI with given title and start value.
	 * 
	 * @param	title
	 * 			The title for this new progress GUI's window.
	 */
	public AutopilotGUI(String title) {
				
		// Initialise progress bar
		this.progressBar = new JProgressBar(0, 100);
        this.progressBar.setValue(0);
        this.progressBar.setStringPainted(false);
        
        // Initialise path planning toggle button
        this.plannerButton = new JCheckBox("Toggle motion planning");
        
        // Initialise panel
        this.panel = new JPanel();
        // this.panel.add(this.progressBar);
        this.panel.add(this.plannerButton);
        this.panel.setOpaque(true); //content panes must be opaque
        
        // Initialise the window and display it
        this.frame = new JFrame(title);
        this.frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.frame.setResizable(false);
        this.frame.setContentPane(panel);
        this.frame.pack();
        this.frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        adjustFrameWidth(this.frame);
        this.frame.setVisible(true);
        
	}
	
	/**
	 * Initialise this new autopilot progress GUI with given start value.
	 * 
	 * @effect	This new autopilot GUI is initialised with the given start value
	 * 			and "Autopilot Progress" as its title
	 */
	public AutopilotGUI() {
		this("Autopilot");
	}
	
	/**
	 * Variable registering this GUI's frame.
	 */
	private JFrame frame;
	
	/**
	 * Variable registering the panel containing the progress bar for this progress GUI.
	 */
	private JPanel panel;
	
	/**
	 * The progress bar for this autopilot panel.
	 */
	private JProgressBar progressBar;
	
	/**
	 * Add an event listener to this GUI.
	 * 
	 * @param 	actionListener
	 * 			The action listener to add.
	 */
	public void addActionListener(ActionListener actionListener) {
		plannerButton.addActionListener(actionListener);
	}
	
	/**
	 * Variable registering a button for this GUI that keeps whether or not a planner is to be used.
	 */
	private JCheckBox plannerButton;
	
	/**
	 * Update this GUI's progress indicator to the given value.
	 * 
	 * @param 	newValue
	 * 			The new progress value for the indicator.
	 */
	public void updateProgress(int newValue) {
		this.progressBar.setValue(newValue);
	}
	
	/**
	 * Return this GUI's progress indicator's current value.
	 * 
	 * @return	The progress currently displayed by this GUI's progress indicator.
	 */
	public int currentProgress() {
		return this.progressBar.getValue();
	}
	
	/**
	 * Adjust the width of the given frame to fit its title.
	 * 
	 * @param 	frame
	 * 			The frame whose width is to be adapted to its title.
	 */
	private static void adjustFrameWidth(JFrame frame) {
		
		// Get title width
		Font defaultFont = UIManager.getDefaults().getFont("Label.font");
	    int titleStringWidth = SwingUtilities.computeStringWidth(new JLabel().getFontMetrics(defaultFont),
	            frame.getTitle()) + 100; // 100 pixels for buttons etc.
	    
	    // Adjust the minimum size
	    Dimension minimumSize = frame.getMinimumSize();
	    minimumSize.width = titleStringWidth;
	    frame.setMinimumSize(minimumSize);
	    
	}
	
}