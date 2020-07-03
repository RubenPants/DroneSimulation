package swing_components;

import java.awt.BorderLayout;
import java.awt.EventQueue;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.List;

import javax.swing.JCheckBoxMenuItem;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPopupMenu;
import javax.swing.KeyStroke;

import org.lwjgl.LWJGLException;
import org.lwjgl.opengl.ContextAttribs;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.PixelFormat;
import org.lwjgl.util.vector.Vector3f;

import interfaces.AutopilotConfig;
import entities.Cube;
import rendering.CameraView;
import rendering.SimulationStatus;
import worldSimulation.Path;

public class MainFrame extends JFrame{

	private TabPane tabPane;
	private CanvasPanel canvasPanel;
	
	private JMenuItem importPathItem;
	private JMenuItem exportPathItem;
	private JMenuItem exportImageItem;
	
	private JMenuItem accuracyItem;
	
	private JMenuItem startItem;
	private JMenuItem pauseItem;
	private JMenuItem resumeItem;
	private JMenuItem restartItem;
	private JMenuItem resetItem;
	
	private CloseListener closeListener;
	private SimulationListener simListener;
	
	public MainFrame(int width, int height){
		super("Virtual testbed");
		
		setLayout(new BorderLayout());
		
        canvasPanel = new CanvasPanel(width, height);
        tabPane = new TabPane();
        
        JMenuBar menuBar = createJMenuBar();
        setJMenuBar(menuBar);
         
        add(tabPane, BorderLayout.EAST);
        add(canvasPanel, BorderLayout.CENTER);
        
        setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
        addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent we){
                int result = JOptionPane.showConfirmDialog(MainFrame.this, "Do you want to quit the Application?");
                if(result == JOptionPane.OK_OPTION){
                	if (closeListener != null)
                		closeListener.requestClose();
                    setVisible(false);
                }
            }
        });
        pack();

        setMinimumSize(getSize());
		setLocationRelativeTo(null);

		ContextAttribs attribs = new ContextAttribs(3,2).withForwardCompatible(true).withProfileCore(true);
		
		try 
		{			
			Display.create(new PixelFormat().withDepthBits(24), attribs);
		} 
		catch (LWJGLException e) 
		{
		    e.printStackTrace();
		}
	}
	
	public void setCloseListener(CloseListener listener) {
		this.closeListener = listener;
	}
	
	public void setSettingsListener(SettingsListener listener) {
		tabPane.setSettingsListener(listener);
	}
	
	public void setSimulationListener(SimulationListener listener) {
		this.simListener = listener;
	}
	
	public void updateOrientationLabels(float heading, float pitch, float roll) {
		tabPane.updateOrientationLabels(heading, pitch, roll);
	}
	
	public void updateVelocityLabels(Vector3f position, Vector3f velocity, Vector3f angularVelocity) {
		tabPane.updateVelocityLabels(position, velocity, angularVelocity);
	}
	
	private JMenuBar createJMenuBar() {
		JMenuBar menuBar = new JMenuBar();
		JMenu fileMenu = new JMenu("File");
		fileMenu.getPopupMenu().setLightWeightPopupEnabled(false); //Forceer het menu heavyweight om overlap met canvas te vermijden
		JMenu settingsMenu = new JMenu("Settings");
		settingsMenu.getPopupMenu().setLightWeightPopupEnabled(false);
		JMenu runMenu = new JMenu("Run");
		runMenu.getPopupMenu().setLightWeightPopupEnabled(false);
		JMenu windowMenu = new JMenu("Window");
		windowMenu.getPopupMenu().setLightWeightPopupEnabled(false);
		menuBar.add(fileMenu);
		menuBar.add(settingsMenu);
		menuBar.add(runMenu);
		menuBar.add(windowMenu);
		
		
		importPathItem = new JMenuItem("Import Path");
		exportPathItem = new JMenuItem("Export Path");
		exportImageItem = new JMenuItem("Export Drone View");
		exportImageItem.setToolTipText("Export the current drone view. This can only be done if the simulation is paused.");
		fileMenu.add(importPathItem);
		fileMenu.add(exportPathItem);
		fileMenu.add(exportImageItem);
		
		JMenuItem droneConfigItem = new JMenuItem("Drone Configurations");
		droneConfigItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				tabPane.setSelectedIndex(1);
			}
		});
		ActionListener testBedSettings = new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				tabPane.setSelectedIndex(0);
			}
		};
		JMenuItem backgroundItem = new JMenuItem("Background Color");
		backgroundItem.addActionListener(testBedSettings);
		JMenuItem fovItem = new JMenuItem("Field of View");
		fovItem.addActionListener(testBedSettings);
		accuracyItem = new JMenuItem("Testbed Accuracy Settings");
		
		settingsMenu.add(droneConfigItem);
		settingsMenu.add(backgroundItem);
		settingsMenu.add(fovItem);
		settingsMenu.addSeparator();
		settingsMenu.add(accuracyItem);
		
		startItem = new JMenuItem("Start Simulation");
		startItem.setToolTipText("Configures the drone and starts the simulation");
		startItem.setAccelerator((KeyStroke.getKeyStroke(KeyEvent.VK_F1, 0)));
		resumeItem = new JMenuItem("Resume Simulation");
		resumeItem.setToolTipText("Resume the simulation");
		resumeItem.setEnabled(false);
		resumeItem.setAccelerator((KeyStroke.getKeyStroke(KeyEvent.VK_F3, 0)));
		pauseItem = new JMenuItem("Pause Simulation");
		pauseItem.setToolTipText("Pause the simulation");
		pauseItem.setEnabled(false);
		pauseItem.setAccelerator((KeyStroke.getKeyStroke(KeyEvent.VK_F2, 0)));
		restartItem = new JMenuItem("Restart Simulation");
		restartItem.setToolTipText("Restart the simulation. The used path is loaded again.");
		restartItem.setAccelerator((KeyStroke.getKeyStroke(KeyEvent.VK_F4, 0)));
		resetItem = new JMenuItem("Reset Simulation");
		resetItem.setToolTipText("Reset the simulation. The last path is deleted.");
		resetItem.setAccelerator((KeyStroke.getKeyStroke(KeyEvent.VK_F5, 0)));

		startItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simListener.swapSimulationStatus(SimulationStatus.ConfigRequested);
				pauseItem.setEnabled(true);
				startItem.setEnabled(false);
			}
		});
		resumeItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simListener.swapSimulationStatus(SimulationStatus.Started);	
				pauseItem.setEnabled(true);
				resumeItem.setEnabled(false);
			}
		});
		pauseItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simListener.swapSimulationStatus(SimulationStatus.Paused);
				resumeItem.setEnabled(true);
				pauseItem.setEnabled(false);
			}
		});
		restartItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simListener.swapSimulationStatus(SimulationStatus.RestartRequested);
			}
		});
		resetItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simListener.swapSimulationStatus(SimulationStatus.ResetRequested);
			}
		});
		runMenu.add(startItem);
		runMenu.add(pauseItem);
		runMenu.add(resumeItem);
		runMenu.add(restartItem);
		runMenu.add(resetItem);
		runMenu.addSeparator();
		
		JMenuItem testItem = new JMenuItem("Run tests");
		testItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simListener.testingRequested();
			}
		});
		runMenu.add(testItem);
		
		JCheckBoxMenuItem sidePanelItem = new JCheckBoxMenuItem("SidePanel");
		sidePanelItem.setSelected(true);
		sidePanelItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				tabPane.setVisible(sidePanelItem.isSelected());
			}
		});
		JMenu cameraMenu = new JMenu("Show Camera View");
		JMenuItem customCameraItem = new JMenuItem("Custom View");
		customCameraItem.setToolTipText("<html>" + "Lock or unlock the camera to the drone with Space Bar."
				+ "<br>" + "When unlocked you can move around freely with QZSD" + "<br>"
				+ "and rotate with the mouse." +"</html>");
		customCameraItem.setAccelerator((KeyStroke.getKeyStroke(KeyEvent.VK_1, ActionEvent.CTRL_MASK)));
		customCameraItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simListener.swapView(CameraView.Custom);
			}
		});
		JMenuItem chaseCameraItem = new JMenuItem("Chase View");
		chaseCameraItem.setAccelerator((KeyStroke.getKeyStroke(KeyEvent.VK_2, ActionEvent.CTRL_MASK)));
		chaseCameraItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simListener.swapView(CameraView.ThirdPerson);
			}
		});
		JMenuItem orthoCameraItem = new JMenuItem("Orthogonal Views");
		orthoCameraItem.setAccelerator((KeyStroke.getKeyStroke(KeyEvent.VK_3, ActionEvent.CTRL_MASK)));
		orthoCameraItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simListener.swapView(CameraView.OrthogonalViews);
			}
		});
		JMenuItem droneCameraItem = new JMenuItem("Drone View");
		droneCameraItem.setAccelerator((KeyStroke.getKeyStroke(KeyEvent.VK_4, ActionEvent.CTRL_MASK)));
		droneCameraItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simListener.swapView(CameraView.DroneView);
			}
		});
		
		cameraMenu.add(customCameraItem);
		cameraMenu.add(chaseCameraItem);
		cameraMenu.add(orthoCameraItem);
		cameraMenu.add(droneCameraItem);
		
		windowMenu.add(cameraMenu);
		windowMenu.add(sidePanelItem);
		
		return menuBar;
	}
	
	public void addPathListener(PathListener listener) {
		tabPane.addPathListener(listener);
	}
	
	public void setPathData(List<Path> paths) {
		tabPane.setPathData(paths);
	}
	
	public void setCubesData(List<Cube> cubes) {
		tabPane.setCubesData(cubes);
	}
	
	public void refreshTables() {
		tabPane.refreshTables();
	}
	
	public void resetRunMenu() {
		 EventQueue.invokeLater(new Runnable() {
		        @Override
		        public void run() {
		    		startItem.setEnabled(true);
		    		pauseItem.setEnabled(false);
		    		resumeItem.setEnabled(false);
		    		restartItem.setEnabled(true);
		    		resetItem.setEnabled(true);
		        }
		    });
	}
	
	public AutopilotConfig getConfigs() {
		return tabPane.getConfigs();
	}
	
	public void addImportListener(ActionListener listener){
		importPathItem.addActionListener(listener);
	}
	
	public void addExportListener(ActionListener listener){
		exportPathItem.addActionListener(listener);
		exportImageItem.addActionListener(listener);
	}	
	
	public void updateTime(float frameTime, float time) {
		tabPane.updateTime(frameTime, time);
	}
	
	public void addAccuracyListener(ActionListener listener) {
		accuracyItem.addActionListener(listener);
	}
	
	public void lockSimulation() {
		startItem.setEnabled(false);
		pauseItem.setEnabled(false);
		resumeItem.setEnabled(false);
		restartItem.setEnabled(false);
		resetItem.setEnabled(false);
	}
	
	public TabPane getTabPane(){
		return tabPane;
	}
}
