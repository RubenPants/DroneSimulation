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

import entities.Airport;
import entities.Camera;
import entities.Cube;
import entities.Drone;
import entities.DroneWithWeels;
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
import rendering.TerrainRenderer;
import rendering.TextureFrameBuffer;
import swing_components.AccuracyFrame;
import swing_components.CloseListener;
import swing_components.MainFrame;
import swing_components.PathListener;
import swing_components.SettingsEvent;
import swing_components.SettingsListener;
import swing_components.SimulationListener;
import swing_components.TesterFrame;
import terrain.ProceduralTerrainLoader;
import terrain.Terrain;
import terrain.TerrainTexture;
import terrain.TerrainTexturePack;
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
	
	private static final int ORTHO_ZOOM_LEVEL = 3; //HET ZOOM LEVEL VAN ORTHOVIEW (1 = VER UITGEZOOMD, HOGER = DICHTERBIJ)
	
	public static final int START_DISPLAY_WIDTH = 800;
	public static final int START_DISPLAY_HEIGHT = 800;
	public static final int RESOLUTION_WIDTH = 200;
	public static final int RESOLUTION_HEIGHT = 200;
	private static int FPS_CAP = 50;
	private static int autopilotCallsPerSecond = 50;
	private static int iterationsPerFrame = 10;
	private static boolean useConstantTime = true;
	private static final int INFO_REFRESH_RATE = 10;
	
	private static float lastFrameTime;
	private static long thisFrameTime;
	private static long lastUpdateTime;
	private static float delta;
	
	private static Renderer renderer;
	private static Renderer planeRenderer;
	private static GuiRenderer guiRenderer;
	private static TerrainRenderer terrainRenderer;
	private static Loader loader;
	
	private static DataController controller = new DataController();
	
	private static Drone drone;
	private static DroneStartSettings startSettings = new DroneStartSettings();
	private static TexturedModel texModel;
	private static TexturedModel shadowTexModel;
	private static Airport airport;
	
	private static World world = new World(drone);
	private static Camera camera = new Camera(drone);
	private static ProceduralTerrainLoader terrainLoader;
	private static float cameraDistance = 50f;
	private static Model cubeModel;
	private static float timeRelativeToReal = 2f;
	private static float newRequestedRelativeTime = 2f;
	
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
		//Initialiseer het swing main en tester paneel.
		createMainFrame(START_DISPLAY_WIDTH, START_DISPLAY_HEIGHT);
		createTesterFrame();
		
		//Initialiseer de loader en renderers en de buffers om naar te renderen
		loader = new Loader();
		renderer = new Renderer();
		planeRenderer = new Renderer();
		guiRenderer = new GuiRenderer(loader);
		terrainRenderer = new TerrainRenderer();
		
		buffer = new FrameBuffer(RESOLUTION_WIDTH, RESOLUTION_HEIGHT);		
		textureTopBuffer = new TextureFrameBuffer(2048,2048);
		textureSideBuffer = new TextureFrameBuffer(2048,2048);

		//Initialiseer de filechooser om paden te lezen
		fileChooser = new JFileChooser();
		PathFileFilter ff = new PathFileFilter();
		fileChooser.addChoosableFileFilter(ff);
		fileChooser.setFileFilter(ff);
		pathReader = new PathReader();
		
		//Initialiseer de drone en de wereld voor simulatie
		configs = mainFrame.getConfigs();
		drone = new DroneWithWeels(configs.getEngineMass(),configs.getWingMass(),configs.getWingX(),
				configs.getTailMass(), configs.getTailSize(), configs.getWingLiftSlope(),
				configs.getHorStabLiftSlope(),configs.getVerStabLiftSlope(),configs.getMaxAOA());
		drone.setGravity(configs.getGravity());
		((DroneWithWeels)drone).setOtherConfigs(mainFrame.getTabPane().getConfigPanel());
		drone.reset(startSettings);
		camera = new Camera(drone);
		world = new World(drone);
		configs = mainFrame.getConfigs();
		
		//Initialiseer het drone model
		cubeModel = loader.loadToVAO(Tools.getCubeVertices(5, 5, 5),
				Cube.CUBE_COLORVALUES,
				Cube.CUBE_INDICES);
		Model planeModel = OBJFileLoader.loadOBJ("plane2", loader);
		Model shadowModel = loader.loadTexturedQuad();
		texModel = new TexturedModel(planeModel.getVaoID(),planeModel.getVertexCount(),loader.loadTexture("plane2"));
		shadowTexModel = new TexturedModel(shadowModel.getVaoID(), shadowModel.getVertexCount(), loader.loadTexture("shadowTexture"));
		drone.addModels(texModel, shadowTexModel);
		
		//Splits het scherm in 2 delen voor de orthogonale projecties
		GuiTexture guiTextureTop = new GuiTexture(textureTopBuffer.getTexture(), new Vector2f(0f,0.5f), new Vector2f(1f,-.5f));
		GuiTexture guiTextureSide = new GuiTexture(textureSideBuffer.getTexture(), new Vector2f(0f,-0.5f), new Vector2f(1f,-.5f));
		GuiTexture guiSplitter = new GuiTexture(loader.loadTexture("splitter"), new Vector2f(0,0), new Vector2f(1f,0.002f));
		
		guis.add(guiTextureTop);
		guis.add(guiTextureSide);
		guis.add(guiSplitter);
		
		//Initialiseer de cube grabber
		cubeGrabber = new CubeGrabber(240,60, world);
		
		//*****************TERRAIN TEXTURES**********************
		
		TerrainTexture backgroundTexture = new TerrainTexture(loader.loadTexture("grass"));
		TerrainTexture rTexture = new TerrainTexture(loader.loadTexture("grass"));
		TerrainTexture gTexture = new TerrainTexture(loader.loadTexture("grass"));
		TerrainTexture bTexture = new TerrainTexture(loader.loadTexture("grass"));
				
		TerrainTexturePack texturePack = new TerrainTexturePack(backgroundTexture,rTexture,gTexture,bTexture);
		TerrainTexture blendMap = new TerrainTexture(loader.loadTexture("blendMap"));
		
		Model terrainModel = Terrain.generateTerrain(loader);
		terrainLoader = new ProceduralTerrainLoader(drone, terrainModel, 25, texturePack, blendMap);

		airport = new Airport(70, 400, new Vector2f(0,-175), loader.loadTexture("tarmac2"), loader);		
		//Initialiseer de frame time en maak mainFrame zichtbaar
		lastFrameTime = getCurrentTime();
		lastUpdateTime = getCurrentTime();
		mainFrame.setVisible(true);
	}
	
	/**
	 * Update the whole app. This includes rendering the image, then if the simulation isn't paused updating the drone.
	 * Then the display is updated so the drone can be evolved over the amount of time that it took to do the last frame.
	 */
	public static void updateApp() {
		long time = getCurrentTime();
		//Haal de simulation status op. (Wordt ook aangepast door swing thread)
		SimulationStatus status = simStatus;
		
		/*
		 * Kijk naar de simulation status:
		 * ResetRequested: Herstart de drone en verwijder het pad
		 * RestartRequested: Herstart enkel de drone en het pad
		 * PathUpdateRequested: Herstart de drone en zet het nieuwe pad
		 */
		switch (status) {
		case ResetRequested:
			reset();
			break;
		case RestartRequested:
			reset();
			world.setPath(pathOnRestart);
			break;
		case PathUpdateRequested:
			reset();
			world.setPath(pathOnRestart);
			break;
		case ConfigsChangeRequested:
			changeConfigs();
			break;
		default:
			break;
		}

		terrainLoader.updateTerrain();
		//Kijk of de tijdversnelling veranderd is
		if(timeRelativeToReal != newRequestedRelativeTime) {
			timeRelativeToReal = newRequestedRelativeTime;
		}
		//Render naar de buffers
		renderForAutopilot();

		//Update het scherm (default FrameBuffer)
		updateDisplay();
		
		//Roep de autopilot op. Vraag nieuwe vliegtuiginstellingen en pas deze aan. Eerste keer: configureer
		if(status == SimulationStatus.Started || status == SimulationStatus.ConfigRequested){
			callAutopilot();
		}

		//Als de simulatie gestart is, simuleer de wereld een tijdstap vooruit en controleer op targets
		if(status == SimulationStatus.Started) {
			try {
				for(int i = 0;i<iterationsPerFrame;i++) {
					if(!useConstantTime)
						drone.timePassed(getUpdateTimeSeconds()*timeRelativeToReal/iterationsPerFrame);
					else
						drone.timePassed((float) ((1.0/autopilotCallsPerSecond)/iterationsPerFrame));
				}
				simulationTime += useConstantTime? (1.0/autopilotCallsPerSecond) : getUpdateTimeSeconds();
				if(isTesting && simulationTime > timeToBeat){
					simStatus = SimulationStatus.ResetRequested;
					// TODO
					testerFrame.testFailed("Test failed: Took too long");
					//Vector3f failedCube = world.getPath().getCubes().get(0).getPosition();
					//testerFrame.testFailed("failed on cube height difference: "+(failedCube.y - 150f));
				}
			} catch (RuntimeException e){
				if(isTesting){
					simStatus = SimulationStatus.ResetRequested;
					testerFrame.testFailed("Test failed: MAX AOA exceeded");
				} else {
					showErrorMessage(e.getMessage());
				}
			}
			refreshInfo();
			checkTargetReached();
		}
		//Synchroniseer de update van de app naar het aantal autopilot calls.
		long timeSoFar = getCurrentTime() - time;
		long desiredTime = (long) ((1000/autopilotCallsPerSecond)/timeRelativeToReal);
		if(timeSoFar < desiredTime){
			try {
				//System.out.println("HAD TO SLEEP");
				Thread.sleep(desiredTime - timeSoFar);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		//System.out.println("TOTAL UPDATE TIME TESTBED:" + getCurrentTime() + "-" + time + "=" + (getCurrentTime()-time));
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
				terrainRenderer.changeSettings(e.getFov());
			}
			public void addCube(Vector3f position) {
				AppManager.addCube(position);	
			}
			@Override
			public void setTime(float time) {
				newRequestedRelativeTime = time;
			}
			@Override
			public void setConfigs(AutopilotConfig config) {
				configs = config;
				simStatus = SimulationStatus.ConfigsChangeRequested;
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
				controller.addPath(RandomWorldGenerator.generateRandomWorld(20, WorldGenerationMode.FINDTILCORRECT, cubeModel));
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
		// TODO
		testerFrame = new TesterFrame(10, 10, 1800);
		testerFrame.addStartButtonListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				simStatus = SimulationStatus.ResetRequested;
				newRequestedRelativeTime = testerFrame.getSpeed();
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
				newRequestedRelativeTime = 1;
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
				newRequestedRelativeTime = 1;
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
	private static void callAutopilot(){
		AutopilotOutputs outputs;
		if(simStatus != SimulationStatus.ConfigRequested) {
			outputs = getAutopilotOutputs();
		} else {
			outputs = configDrone();
			pathOnRestart = world.getPath();
			simStatus = SimulationStatus.Started;
			autopilot.setPath(pathOnRestart.getAutopilotPath());
		}
		changeDroneInputs(outputs);
	}
	
	/**
	 * Update the display. The display is synchronised to FPS_CAP frames per second (if it can handle it). 
	 * The amount of time between the last frame and the current frame is saved in delta. 
	 */
	private static void updateDisplay() {
		long currentUpdateTime = getCurrentTime();
		delta = (currentUpdateTime - lastUpdateTime)/1000f;
		lastUpdateTime = currentUpdateTime;
		thisFrameTime += delta*1000;
		if(thisFrameTime >= 1000/FPS_CAP) {
			renderTestbedView();
			lastFrameTime = thisFrameTime/1000f;
			thisFrameTime = 0;
			Display.update();
		}
	}
	
	/**
	 * Get the last frame's time.
	 */
	public static float getUpdateTimeSeconds() {
		return delta;
	}
	
	public static float getFrameTimeSeconds(){
		return lastFrameTime;
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
		boolean targetReached = world.targetReached();
		if(drone.getVelocity().length() < 1 && (drone.getPosition().lengthSquared() - Math.pow(drone.getPosition().y, 2) < 9) &&
				simulationTime > 60) {
			if(targetReached) {
				if(isTesting){
					testerFrame.appendText("Test completed");
					testerFrame.testCompleted(simulationTime);
					simStatus = SimulationStatus.ResetRequested;
				}
				else {
		            simStatus = SimulationStatus.Paused;
					showMessage("Congratulations, your autopilot completed the path and stopped at its initial position.");
				}
			} else {
				if(isTesting){
					testerFrame.testFailed("Landed, but not all cubes done");
					simStatus = SimulationStatus.ResetRequested;
				}
				else {
		            simStatus = SimulationStatus.Paused;
					showErrorMessage("Your autopilot landed, but didn't get all the cubes");
				}
			}
		}
	}
	
	public static boolean closeRequested(){
		return shouldClose;
	}
	
	//Current time in milliseconds
	private static long getCurrentTime() {
		return Sys.getTime()*1000/Sys.getTimerResolution();
	}
	
	public static Model loadToVAO(float[] positions, float[] colors, int[] indices) {
		return loader.loadToVAO(positions, colors, indices);
	}
	
	private static void renderForAutopilot(){
		buffer.bindFrameBuffer();
		planeRenderer.prepareRender();
		planeRenderer.renderCubes(world, camera, 1);
		terrainRenderer.calculatePerspectiveMatrix();
		terrainRenderer.render(terrainLoader.getTerrains(), camera);
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
	}
	
	private static void renderTestbedView() {
		renderer.prepareRender();
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
			renderer.prepareRender();
			Camera orthoTopCamera = new Camera(new Vector3f(drone.getPosition().x,5000,drone.getPosition().z), (float)Math.PI/2,-(float)Math.PI/2,0);
			//Camera orthoTopCamera = new Camera(new Vector3f(0,3000,0), (float)Math.PI/2,-(float)Math.PI/2,0);
			renderer.calculateOrthogonalMatrix(5000/ORTHO_ZOOM_LEVEL, 2500/ORTHO_ZOOM_LEVEL);
			terrainRenderer.calculateOrthogonalMatrix(5000/ORTHO_ZOOM_LEVEL, 2500/ORTHO_ZOOM_LEVEL);
			terrainRenderer.render(terrainLoader.getTerrains(), orthoTopCamera);
			renderer.renderTex(airport, orthoTopCamera, 1);
			renderer.renderShadow(drone, orthoTopCamera);
			renderer.renderTex(drone, orthoTopCamera, 12f/ORTHO_ZOOM_LEVEL);
			renderer.renderCubes(world, orthoTopCamera, 12/ORTHO_ZOOM_LEVEL);
			textureTopBuffer.unbindCurrentFrameBuffer();
			
			textureSideBuffer.bindFrameBuffer();
			renderer.prepareRender();
			Camera orthoSideCamera = new Camera(new Vector3f(5000,drone.getPosition().y,drone.getPosition().z), (float)Math.PI/2,0,0);
			renderer.renderCubes(world, orthoSideCamera, 12/ORTHO_ZOOM_LEVEL);
			renderer.renderTex(drone, orthoSideCamera, 12f/ORTHO_ZOOM_LEVEL);
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
			terrainRenderer.calculatePerspectiveMatrix();
			renderer.renderCubes(world, thirdPersonCamera, 1);
			renderer.renderTex(drone, thirdPersonCamera);
			renderer.renderTex(airport, thirdPersonCamera, 1);
			terrainRenderer.render(terrainLoader.getTerrains(), thirdPersonCamera);
			renderer.renderShadow(drone, thirdPersonCamera);

			break;
		case Custom:
			customCamera.move();
			customCamera.moveAroundDrone(drone);
			renderer.calculatePerspectiveMatrix();
			terrainRenderer.calculatePerspectiveMatrix();
			renderer.renderCubes(world, customCamera, 1);
			renderer.renderTex(drone, customCamera);
			terrainRenderer.render(terrainLoader.getTerrains(), customCamera);
			renderer.renderTex(airport, customCamera, 1);
			renderer.renderShadow(drone, customCamera);
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
	
	private static void changeDroneInputs(AutopilotOutputs outputs){
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
	            int i = JOptionPane.showConfirmDialog(null, message + " Press 'Yes' to quit the app, or press 'No' to restart.");
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
		deltaLastInfoUpdate += getUpdateTimeSeconds();
		if(deltaLastInfoUpdate >= 1.0/INFO_REFRESH_RATE) {
			mainFrame.updateOrientationLabels(drone.getHeading(), drone.getPitch(), drone.getRoll());
			mainFrame.updateVelocityLabels(drone.getPosition(), drone.getVelocity(), drone.getRelativeAngularVelocity());
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
		mainFrame.updateVelocityLabels(drone.getPosition(), drone.getVelocity(), drone.getRelativeAngularVelocity());
		if(isTesting){
			testsSoFar++;
			if(testsSoFar > testAmount){
				testsSoFar = 0;
				isTesting = false;
			} else {
				// TODO
				//world.setPath(RandomWorldGenerator.generateRandomWorld(20, WorldGenerationMode.FINDTILCORRECT, cubeModel));
				world.setPath(pathReader.ReadAndCreatePathFromText(new File("res/pad"+Integer.toString(testsSoFar)+".txt"), cubeModel));
				simStatus = SimulationStatus.ConfigRequested;
			}
		} else {
			mainFrame.resetRunMenu();
		}
	}
	
	private static void changeConfigs(){
		drone = new DroneWithWeels(configs.getEngineMass(),configs.getWingMass(),configs.getWingX(),
				configs.getTailMass(), configs.getTailSize(), configs.getWingLiftSlope(),
				configs.getHorStabLiftSlope(),configs.getVerStabLiftSlope(),configs.getMaxAOA());
		drone.setGravity(configs.getGravity());
		drone.reset(startSettings);
		drone.addModels(texModel, shadowTexModel);
		terrainLoader.setDrone(drone);
		world.setDrone(drone);
		camera = new Camera(drone);
		simStatus = SimulationStatus.Idle;
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
		BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
		for (int i=0 ; i<height ; i++) {
			for (int j=0 ; j<width ; j++) {
				byte r = imageCopy[(i * width + j) * 3 + 0];
				byte g = imageCopy[(i * width + j) * 3 + 1];
				byte b = imageCopy[(i * width + j) * 3 + 2];
				int rgb = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
				bufferedImage.setRGB(j, i, rgb);
			}
		}
		
		try {
			ImageIO.write(bufferedImage, "bmp", file);
		} catch (IOException e) {
			e.printStackTrace();
		}		
	}
}
