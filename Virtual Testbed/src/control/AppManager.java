package control;

import java.awt.EventQueue;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.JFileChooser;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;

import org.lwjgl.Sys;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL30;
import org.lwjgl.util.vector.Vector2f;
import org.lwjgl.util.vector.Vector3f;

import entities.Camera;
import entities.Cube;
import entities.Drone;
import entities.Model;
import entities.TexturedModel;
import guis.GuiRenderer;
import guis.GuiTexture;
import interfaces.Autopilot;
import interfaces.AutopilotConfig;
import interfaces.AutopilotInputs;
import interfaces.AutopilotOutputs;
import objConverter.OBJFileLoader;
import rendering.CameraView;
import rendering.FrameBuffer;
import rendering.Loader;
import rendering.Renderer;
import rendering.SimulationStatus;
import rendering.TextureFrameBuffer;
import swing_components.AccuracyFrame;
import swing_components.CloseListener;
import swing_components.MainFrame;
import swing_components.PathListener;
import swing_components.SettingsEvent;
import swing_components.SettingsListener;
import swing_components.SimulationListener;
import swing_components.TesterFrame;
import tools.CubeGrabber;
import tools.PathFileFilter;
import tools.Tools;
import worldSimulation.DroneStartSettings;
import worldSimulation.Path;
import worldSimulation.PathReader;
import worldSimulation.RandomWorldGenerator;
import worldSimulation.World;
import worldSimulation.WorldGenerationMode;

public class AppManager {
	
	public static final int START_DISPLAY_WIDTH = 800;
	public static final int START_DISPLAY_HEIGHT = 800;
	public static final int RESOLUTION_WIDTH = 200;
	public static final int RESOLUTION_HEIGHT = 200;
	private static int FPS_CAP = 100;
	private static int autopilotCallsPerSecond = 100;
	private static int iterationsPerFrame = 10;
	private static boolean useConstantTime = false;
	private static final int INFO_REFRESH_RATE = 10;
	
	private static long lastFrameTime;
	private static float delta;
	
	private static Renderer renderer;
	private static Renderer planeRenderer;
	private static GuiRenderer guiRenderer;
	private static Loader loader;
	
	private static DataController controller = new DataController();
	
	private static Drone drone;
	private static DroneStartSettings startSettings = new DroneStartSettings();
	private static Model planeModel;
	private static TexturedModel texModel;
	
	private static World world = new World(drone);
	private static Camera camera = new Camera(drone);
	private static float cameraDistance = .8f;
	private static Model cubeModel;
	private static float timeRelativeToReal = 1f;
	private static float newRequestedTime = 1f;
	
	private static FrameBuffer buffer;
	private static TextureFrameBuffer textureTopBuffer;
	private static TextureFrameBuffer textureSideBuffer;
	private static List<GuiTexture> guis = new ArrayList<GuiTexture>();	
	private static ByteBuffer imageBuffer = ByteBuffer.allocateDirect(RESOLUTION_WIDTH*RESOLUTION_HEIGHT*3);
	private static byte[] image;
	private static byte[] imageCopy;
	private static boolean imageCopyRequested = true;
	private static Camera customCamera = new Camera(new Vector3f(0,5,5), 0, 0, 0);
	
	private static MainFrame mainFrame;
	private static JFileChooser fileChooser;
	private static PathReader pathReader;
	private static CubeGrabber cubeGrabber;
	
	private static TesterFrame testerFrame;
	private static boolean isTesting = false;
	private static int testAmount;
	private static int testsSoFar = 0;
	private static float timeToBeat;
	
	private static float deltaLastInfoUpdate = 0; 
	private static boolean shouldClose = false;
	private static CameraView cameraView = CameraView.ThirdPerson;
	private static SimulationStatus simStatus = SimulationStatus.Idle;
	
	private static Path pathOnRestart = new Path();
	private static float simulationTime = 0;
	
	private static Autopilot autopilot;
	private static AutopilotConfig configs;
	
	/**
	 * Starts the app by creating and showing the configurations frame. The mainframe is made as well but is still invisible.
	 */
	public static void createApp() {
		createMainFrame(START_DISPLAY_WIDTH, START_DISPLAY_HEIGHT);
		createTesterFrame();
		
		loader = new Loader();
		renderer = new Renderer();
		planeRenderer = new Renderer();
		guiRenderer = new GuiRenderer(loader);

		fileChooser = new JFileChooser();
		PathFileFilter ff = new PathFileFilter();
		fileChooser.addChoosableFileFilter(ff);
		fileChooser.setFileFilter(ff);
		pathReader = new PathReader();
		
		configs = mainFrame.getConfigs();
		drone = new Drone(configs.getEngineMass(),configs.getWingMass(),configs.getWingX(),
				configs.getTailMass(), configs.getTailSize(), configs.getWingLiftSlope(),
				configs.getHorStabLiftSlope(),configs.getVerStabLiftSlope(),configs.getMaxAOA());
		drone.setGravity(configs.getGravity());
		drone.reset(startSettings);
		camera = new Camera(drone);
		world = new World(drone);
		configs = mainFrame.getConfigs();
		
		buffer = new FrameBuffer(RESOLUTION_WIDTH, RESOLUTION_HEIGHT);
		cubeModel = loader.loadToVAO(Tools.getCubeVertices(1, 1, 1),
				Cube.CUBE_COLORVALUES,
				Cube.CUBE_INDICES);
		planeModel = OBJFileLoader.loadOBJ("test", loader);
		texModel = new TexturedModel(planeModel.getVaoID(),planeModel.getVertexCount(),loader.loadTexture("testdonker"));
		drone.addModel(texModel);
		
		textureTopBuffer = new TextureFrameBuffer(2048,2048);
		textureSideBuffer = new TextureFrameBuffer(2048,2048);
		
		GuiTexture guiTextureTop = new GuiTexture(textureTopBuffer.getTexture(), new Vector2f(0f,0.5f), new Vector2f(1f,-.5f));
		GuiTexture guiTextureSide = new GuiTexture(textureSideBuffer.getTexture(), new Vector2f(0f,-0.5f), new Vector2f(1f,-.5f));
		GuiTexture guiSplitter = new GuiTexture(loader.loadTexture("splitter"), new Vector2f(0,0), new Vector2f(1f,0.002f));
		
		guis.add(guiTextureTop);
		guis.add(guiTextureSide);
		guis.add(guiSplitter);
		
		cubeGrabber = new CubeGrabber(240,60, world);
		
		lastFrameTime = getCurrentTime();
		mainFrame.setVisible(true);
	}
	
	/**
	 * Create the mainframe, including all the openGL stuff. (Renderer en loader initialised as well)
	 */
	private static void createMainFrame(int width, int height) {
		mainFrame = new MainFrame(width, height);
		mainFrame.setCloseListener(new CloseListener() {
			public void requestClose() {
				shouldClose = true;
				mainFrame.dispose();
			}
		});
		mainFrame.setSettingsListener(new SettingsListener() {
			public void changeRenderSettings(SettingsEvent e) {
				renderer.changeSettings(e.getFov(), e.getRed(), e.getGreen(), e.getBlue());
			}
			public void addCube(Vector3f position) {
				AppManager.addCube(position);	
			}
			@Override
			public void setTime(float time) {
				newRequestedTime = time;
			}
			@Override
			public void setConfigs(AutopilotConfig config) {
				configs = config;
				simStatus = SimulationStatus.SettingsChangeRequested;
			}
			@Override
			public void setStartSettigs(DroneStartSettings settings) {
				startSettings = settings;
			}
			
		});
		mainFrame.setSimulationListener(new SimulationListener() {
			public void swapView(CameraView view) {
				cameraView = view;
			}
			public void swapSimulationStatus(SimulationStatus status) {
				simStatus = status;
				imageCopyRequested = true;
			}
			@Override
			public void testingRequested() {
				testerFrame.setVisible(true);
			}
		});
		
		mainFrame.addPathListener(new PathListener() {
			public void pathDeleted(int index) {
				controller.removePath(index);
			}
			public void newPathCreated() {
				controller.createNewPath();
			}
			public void cubeRemoved(int cubeIndex, int pathIndex) {
				controller.removeCube(cubeIndex, pathIndex);
			}
			public void cubeAdded(Vector3f position, int pathNb) {
				controller.addCube(new Cube(cubeModel,(float)Math.random(),(float)Math.random(),position), pathNb);
			}
			public void loadPathToWorld(int pathIndex) {
				pathOnRestart = controller.getImportedPaths().get(pathIndex);
				simStatus = SimulationStatus.PathUpdateRequested;
			}
			@Override
			public void generateRandomPath() {
				controller.addPath(RandomWorldGenerator.generateRandomWorld(5, WorldGenerationMode.FINDTILCORRECT, cubeModel));
			}
		});
		mainFrame.setPathData(controller.getImportedPaths());
		mainFrame.addImportListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if(fileChooser.showOpenDialog(mainFrame) == JFileChooser.APPROVE_OPTION){
					File selectedFile = fileChooser.getSelectedFile();
					controller.addPath(pathReader.ReadAndCreatePathFromText(selectedFile, cubeModel));
					mainFrame.refreshTables();
				}
			}
		});
		mainFrame.addExportListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if(fileChooser.showSaveDialog(mainFrame) == JFileChooser.APPROVE_OPTION){
					File selectedFile = fileChooser.getSelectedFile();
					if(((JMenuItem)e.getSource()).getText() == "Export Path"){
						if (PathFileFilter.getFileExtension(selectedFile.getName()) == null){
							selectedFile = new File(selectedFile + ".txt");
						}
						pathReader.writePathToTextFile(selectedFile, world.getPath());
					} else if(((JMenuItem)e.getSource()).getText() == "Export Drone View"){
						if (PathFileFilter.getFileExtension(selectedFile.getName()) == null){
							selectedFile = new File(selectedFile + ".bmp");
						}
						saveImage(selectedFile);
					}
				}
			}
		});
		mainFrame.addAccuracyListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
			    EventQueue.invokeLater(new Runnable() {
			        public void run() {
			            AccuracyFrame frame = new AccuracyFrame(autopilotCallsPerSecond, iterationsPerFrame, useConstantTime);
			            frame.addButtonListener(new ActionListener() {
							public void actionPerformed(ActionEvent e) {
								int[] settings = frame.getSettings();
								AppManager.autopilotCallsPerSecond = settings[0];
								AppManager.FPS_CAP = (int) (autopilotCallsPerSecond*timeRelativeToReal);
								AppManager.iterationsPerFrame = settings[1];
								AppManager.useConstantTime = (settings[2] == 1);
								frame.dispose();
							}
						});
			        }
			    });
			}
		});
	}
	
	private static void createTesterFrame(){
		testerFrame = new TesterFrame(100, 8, 180);
		testerFrame.addStartButtonListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simStatus = SimulationStatus.ResetRequested;
				newRequestedTime = testerFrame.getSpeed();
				isTesting = true;
				testAmount = testerFrame.getAmountOfTests();
				timeToBeat = testerFrame.getTimeToBeat();
				testsSoFar = 0;
				testerFrame.lockTesting(true);
				mainFrame.lockSimulation();
			}
		});
		testerFrame.addStopButtonListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simStatus = SimulationStatus.ResetRequested;
				newRequestedTime = 1;
				isTesting = false;
				testerFrame.reset();
				mainFrame.resetRunMenu();
				testerFrame.lockTesting(false);
				timeRelativeToReal = 1;
				testerFrame.setVisible(false);
			}
		});
		testerFrame.addPauseButtonListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if(simStatus == SimulationStatus.Paused) {
					simStatus = SimulationStatus.Started;
					testerFrame.paused(false);
				} else {
					simStatus = SimulationStatus.Paused;
					testerFrame.paused(true);
				}
			}
		});
		testerFrame.addClearButtonListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simStatus = SimulationStatus.ResetRequested;
				newRequestedTime = 1;
				isTesting = false;
				testerFrame.reset();
				testerFrame.lockTesting(false);
				timeRelativeToReal = 1;
			}
		});
	}
	
	/**
	 * Update the virtual drone. (Ask the autopilot for outputs and change the settings of the plane.)
	 * If the drone is not yet configured for the autopilot, then it gets configured. 
	 */
	private static void updateDrone(){
		AutopilotOutputs outputs;
		if(simStatus != SimulationStatus.ConfigRequested) {
			outputs = getAutopilotOutputs();
		} else {
			outputs = configDrone();
			pathOnRestart = world.getPath();
			simStatus = SimulationStatus.Started;
		}
		changeDroneSettings(outputs);
	}
	
	/**
	 * Update the display. The display is synchronised to FPS_CAP frames per second (if it can handle it). 
	 * The amount of time between the last frame and the current frame is saved in delta. 
	 */
	private static void updateDisplay() {
		Display.sync(FPS_CAP);
		Display.update();
		long currentFrameTime = getCurrentTime();
		delta = (currentFrameTime - lastFrameTime)/1000f;
		lastFrameTime = currentFrameTime;
	}
	
	/**
	 * Update the whole app. This includes rendering the image, then if the simulation isn't paused updating the drone.
	 * Then the display is updated so the drone can be evolved over the amount of time that it took to do the last frame.
	 */
	public static void updateApp() {
		SimulationStatus status = simStatus;
		if(status == SimulationStatus.ResetRequested){
			reset();
		}
		if(status == SimulationStatus.PathUpdateRequested || status == SimulationStatus.RestartRequested) {
			reset();
			world.setPath(pathOnRestart);
		}
		if(status == SimulationStatus.SettingsChangeRequested){
			drone = new Drone(configs.getEngineMass(),configs.getWingMass(),configs.getWingX(),
					configs.getTailMass(), configs.getTailSize(), configs.getWingLiftSlope(),
					configs.getHorStabLiftSlope(),configs.getVerStabLiftSlope(),configs.getMaxAOA());
			drone.setGravity(configs.getGravity());
			drone.reset(startSettings);
			drone.addModel(texModel);
			world.setDrone(drone);
			camera = new Camera(drone);
			simStatus = SimulationStatus.Idle;
		}
		if(timeRelativeToReal != newRequestedTime) {
			timeRelativeToReal = newRequestedTime;
			FPS_CAP = (int) (autopilotCallsPerSecond*newRequestedTime);
		}
		render();
		if(status == SimulationStatus.Started || status == SimulationStatus.ConfigRequested){
			updateDrone();
		}
		updateDisplay();
		if(status == SimulationStatus.Started) {
			try {
				for(int i = 0;i<iterationsPerFrame;i++) {
					if(!useConstantTime)
						drone.timePassed(getFrameTimeSeconds()*timeRelativeToReal/iterationsPerFrame);
					else
						drone.timePassed((float) ((1.0/FPS_CAP)*timeRelativeToReal/iterationsPerFrame));
				}
				simulationTime += useConstantTime? (1.0/FPS_CAP)*timeRelativeToReal : getFrameTimeSeconds()*timeRelativeToReal;
				if(isTesting && simulationTime > timeToBeat){
					simStatus = SimulationStatus.ResetRequested;
					testerFrame.appendText("Test failed: Took too long");
					testerFrame.testFailed();
				}
			} catch (RuntimeException e){
				if(isTesting){
					simStatus = SimulationStatus.ResetRequested;
					testerFrame.appendText("Test failed: MAX AOA exceeded");
					testerFrame.testFailed();
				} else {
					showErrorMessage(e.getMessage());
				}
			}
			refreshInfo();
			checkTargetReached();
		}
	}
	
	/**
	 * Get the last frame's time.
	 */
	public static float getFrameTimeSeconds() {
		return delta;
	}
	
	/**
	 * Close the app. Cleans up all the data stored in the loader, renderer and buffers. Destroys the display.
	 */
	public static void closeApp() {
		loader.cleanUp();
		renderer.cleanUp();
		planeRenderer.cleanUp();
		guiRenderer.cleanUp();
		buffer.cleanUp();
		textureTopBuffer.cleanUp();
		textureSideBuffer.cleanUp();
		Display.destroy();
	}

	private static void checkTargetReached() {
		if(world.targetReached()) {
			if(isTesting){
				testerFrame.appendText("Test completed");
				testerFrame.testCompleted(simulationTime);
				simStatus = SimulationStatus.ResetRequested;
			}
			else {
	            simStatus = SimulationStatus.Paused;
				showMessage("Congratulations, your autopilot reached the target");
			}
		}
	}
	
	public static boolean closeRequested(){
		return shouldClose;
	}
	
	private static long getCurrentTime() {
		return Sys.getTime()*1000/Sys.getTimerResolution();
	}
	
	public static Model loadToVAO(float[] positions, float[] colors, int[] indices) {
		return loader.loadToVAO(positions, colors, indices);
	}
	
	private static void render() {
		buffer.bindFrameBuffer();
		planeRenderer.renderCubes(world, camera, 1);
		imageBuffer.clear();
		GL11.glReadPixels(0, 0, RESOLUTION_WIDTH, RESOLUTION_HEIGHT, GL11.GL_RGB, GL11.GL_UNSIGNED_BYTE, imageBuffer);
		byte[] image_flipped = new byte[RESOLUTION_WIDTH*RESOLUTION_HEIGHT*3];
		imageBuffer.get(image_flipped);
		image = new byte[RESOLUTION_WIDTH*RESOLUTION_HEIGHT*3];
		
		for(int x = 0; x<RESOLUTION_WIDTH;x++){
			for(int y = 0; y<RESOLUTION_HEIGHT;y++){
				image[(x+RESOLUTION_WIDTH*y)*3] = image_flipped[(x+(RESOLUTION_HEIGHT-1-y)*RESOLUTION_WIDTH)*3];
				image[(x+RESOLUTION_WIDTH*y)*3 + 1] = image_flipped[(x+(RESOLUTION_HEIGHT-1-y)*RESOLUTION_WIDTH)*3 + 1];
				image[(x+RESOLUTION_WIDTH*y)*3 + 2] = image_flipped[(x+(RESOLUTION_HEIGHT-1-y)*RESOLUTION_WIDTH)*3 + 2];
			}	
		}
		if(imageCopyRequested) {
			imageCopy = image.clone();
			imageCopyRequested = false;
		}
		
		buffer.unbindCurrentFrameBuffer();
		switch(cameraView) {
		case DroneView:
			buffer.bindFrameBuffer();
			GL30.glBindFramebuffer(GL30.GL_DRAW_FRAMEBUFFER, 0);
			GL30.glBlitFramebuffer(
				0, 0, RESOLUTION_WIDTH, RESOLUTION_HEIGHT,
				0, 0, Display.getWidth(), Display.getHeight(),
				GL11.GL_COLOR_BUFFER_BIT, GL11.GL_NEAREST);
			buffer.unbindCurrentFrameBuffer();
			break;
		case OrthogonalViews:
			textureTopBuffer.bindFrameBuffer();
			Camera orthoTopCamera = new Camera(new Vector3f(0,150,-100), (float)Math.PI/2,-(float)Math.PI/2,0);
			renderer.calculateOrthogonalMatrix(240, 60);
			renderer.renderCubes(world, orthoTopCamera, 2);
			renderer.renderTex(drone, orthoTopCamera, 1f);
			textureTopBuffer.unbindCurrentFrameBuffer();
			
			textureSideBuffer.bindFrameBuffer();
			Camera orthoSideCamera = new Camera(new Vector3f(150,0,-100), (float)Math.PI/2,0,0);
			renderer.renderCubes(world, orthoSideCamera, 2);
			renderer.renderTex(drone, orthoSideCamera, 1f);
			textureSideBuffer.unbindCurrentFrameBuffer();

			guiRenderer.render(guis);

			cubeGrabber.update(orthoTopCamera, orthoSideCamera);
			
			break;
		case ThirdPerson:
			Camera thirdPersonCamera = new Camera(
					Vector3f.add(drone.getPosition(),
					new Vector3f((float) (cameraDistance*Math.sin(drone.getHeading())),
							0,(float) (cameraDistance*Math.cos(drone.getHeading()))),null),
					drone.getHeading(), 0, 0);
			renderer.calculatePerspectiveMatrix();
			renderer.renderCubes(world, thirdPersonCamera, 1);
			renderer.renderTex(drone, thirdPersonCamera);
			break;
		case Custom:
			customCamera.move();
			customCamera.moveAroundDrone(drone);
			renderer.calculatePerspectiveMatrix();
			renderer.renderCubes(world, customCamera, 1);
			renderer.renderTex(drone, customCamera);
			break;
		}
	}
	
	public static void addCube(Vector3f position) {
		world.addCube(new Cube(cubeModel, (float)Math.random(), (float) (1-Math.random()*0.7), position));
	}
	
	private static AutopilotOutputs configDrone(){
		
		AutopilotInputs inputs = new AutopilotInputs(){
			public float getElapsedTime() {
				return simulationTime;
			}
			public float getHeading() {
				return drone.getHeading();
			}
			public byte[] getImage() {
				return image;
			}
			public float getPitch() {
				return drone.getPitch();
			}
			public float getRoll() {
				return drone.getRoll();
			}
			public float getX() {
				return drone.getPosition().getX();
			}
			public float getY() {
				return drone.getPosition().getY();
			}
			public float getZ() {
				return drone.getPosition().getZ();
			}
		};
		return autopilot.simulationStarted(configs, inputs);
	}
	
	private static AutopilotOutputs getAutopilotOutputs(){
		AutopilotInputs inputs = new AutopilotInputs(){
			public float getElapsedTime() {
				return simulationTime;
			}
			public float getHeading() {
				return drone.getHeading();
			}
			public byte[] getImage() {
				return image;
			}
			public float getPitch() {
				return drone.getPitch();
			}
			public float getRoll() {
				return drone.getRoll();
			}
			public float getX() {
				return drone.getPosition().getX();
			}
			public float getY() {
				return drone.getPosition().getY();
			}
			public float getZ() {
				return drone.getPosition().getZ();
			}
		};
		return autopilot.timePassed(inputs);
	}
	
	private static void changeDroneSettings(AutopilotOutputs outputs){
		drone.setInputs(outputs);
	}

	public static void setAutopilot(Autopilot autopilot) {
		AppManager.autopilot = autopilot;
	}
	
	private static void showMessage(String message) {
		System.out.println();
		System.out.println("GAME OVER!");
	    EventQueue.invokeLater(new Runnable() {
	        @Override
	        public void run() {
	            int i = JOptionPane.showConfirmDialog(null, "Congratulations, you reached the target! Press 'Yes' to quit the app, or press 'No' to restart.");
	            if(i == JOptionPane.OK_OPTION) {
		            shouldClose = true;
		            mainFrame.dispose();
	            }
	            if (i == JOptionPane.NO_OPTION) {
	            	simStatus = SimulationStatus.ResetRequested;
	            }
	        }
	    });
	}
	
	private static void showErrorMessage(String message) {
    	simStatus = SimulationStatus.ResetRequested;
		System.out.println();
		System.out.println("GAME OVER!");
	    EventQueue.invokeLater(new Runnable() {
	        @Override
	        public void run() {
	        	JOptionPane.showMessageDialog(mainFrame,
	        		    message,
	        		    "ERROR",
	        		    JOptionPane.ERROR_MESSAGE);
	        }
	    });
	}

	private static void refreshInfo() {
		deltaLastInfoUpdate += getFrameTimeSeconds();
		if(deltaLastInfoUpdate >= 1.0/INFO_REFRESH_RATE) {
			mainFrame.updateOrientationLabels(drone.getHeading(), drone.getPitch(), drone.getRoll());
			mainFrame.updateVelocityLabels(drone.getVelocity(), drone.getRelativeAngularVelocity());
			mainFrame.updateTime(getFrameTimeSeconds(), simulationTime);
			deltaLastInfoUpdate = 0;
		}
	}
	
	private static void reset(){
		world.reset();
		drone.reset(startSettings);
		simulationTime = 0;
		simStatus = SimulationStatus.Idle;
		mainFrame.updateOrientationLabels(drone.getHeading(), drone.getPitch(), drone.getRoll());
		mainFrame.updateVelocityLabels(drone.getVelocity(), drone.getRelativeAngularVelocity());
		if(isTesting){
			testsSoFar++;
			if(testsSoFar > testAmount){
				testsSoFar = 0;
				isTesting = false;
			} else {
				world.setPath(RandomWorldGenerator.generateRandomWorld(5, WorldGenerationMode.FINDTILCORRECT, cubeModel));
				simStatus = SimulationStatus.ConfigRequested;
			}
		} else {
			mainFrame.resetRunMenu();
		}
	}
	
	public static void loadPath(String fileName) {
		Path newPath = pathReader.ReadAndCreatePathFromText(new File(fileName), cubeModel);
		newPath.controlColors();
		controller.addPath(newPath);
		pathOnRestart = controller.getImportedPaths().get(0);
		world.setPath(pathOnRestart);
		mainFrame.refreshTables();
	}
	
	public static void saveImage(File file) {
		
		int width = 200, height = 200;
		
		// Convert to image
		BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
		for (int i=0 ; i<height ; i++) {
			for (int j=0 ; j<width ; j++) {
				byte r = image[(i * width + j) * 3 + 0];
				byte g = image[(i * width + j) * 3 + 1];
				byte b = image[(i * width + j) * 3 + 2];
				int rgb = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
				bufferedImage.setRGB(j, height - 1 - i, rgb);
			}
		}
		
		try {
			ImageIO.write(bufferedImage, "bmp", file);
		} catch (IOException e) {
			e.printStackTrace();
		}		
	}
}
