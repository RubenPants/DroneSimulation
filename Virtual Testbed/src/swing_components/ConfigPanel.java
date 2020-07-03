package swing_components;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;

import org.lwjgl.util.vector.Vector3f;

import interfaces.AutopilotConfig;
import worldSimulation.DroneStartSettings;

public class ConfigPanel extends JPanel{

	private JLabel text = new JLabel("Choose your drone configurations and start settings");
	
	private ValuePanel gravityPanel;
	private ValuePanel wingXPanel;
	private ValuePanel tailSizePanel;
	private ValuePanel engineMassPanel;
	private ValuePanel wingMassPanel;
	private ValuePanel tailMassPanel;
	private ValuePanel maxThrustPanel;
	private ValuePanel maxAOAPanel;
	private JCheckBox degreesBox;
	private ValuePanel wingSlopePanel;
	private ValuePanel horStabSlopePanel;
	private ValuePanel verStabSlopePanel;
	private ValuePanel horFOVPanel;
	private ValuePanel verFOVPanel;
	private ValuePanel horResPanel;
	private ValuePanel verResPanel;
	
	private JButton confirmButton;
	
	private ValuePanel startX;
	private ValuePanel startY;
	private ValuePanel startZ;
	private ValuePanel startHeading;
	private ValuePanel startPitch;
	private ValuePanel startRoll;
	
	private SettingsListener settingsListener;
	
	private float wheelY = -4.5f;
	private float frontZ = -8.2f;
	private float rearZ = 2f;
	private float rearX = 4.55f;
	private float tyreSlope = 100f;
	private float dampSlope = 100f;
	private float tyreRadius = 0.2f;
	private float RMax = 2500;
	private float FcMax = 0.91f;
	
	public ConfigPanel() {
		
		gravityPanel = new ValuePanel("Gravity (positive):", 9.81f);
		wingXPanel = new ValuePanel("WingX:", 4.2f);
		tailSizePanel = new ValuePanel("Tail Size:", 4.2f);
		engineMassPanel = new ValuePanel("Engine Mass:", 180f);
		wingMassPanel = new ValuePanel("Wing Mass:", 100f);
		tailMassPanel = new ValuePanel("Tail Mass:", 100f);
		maxThrustPanel = new ValuePanel("Max Thrust:", 2000);
		maxAOAPanel = new ValuePanel("Max AOA:", .261f);
		wingSlopePanel = new ValuePanel("Wing LiftSlope:", 10f);
		horStabSlopePanel = new ValuePanel("Horizontal Stab LiftSlope:", 5f);
		verStabSlopePanel = new ValuePanel("Vertical Stab LiftSlope:", 5f);
		horFOVPanel = new ValuePanel("Horizontal Field of View:", (float)Math.toRadians(120));
		verFOVPanel = new ValuePanel("Vertical Field of View:", (float)Math.toRadians(120));
		horResPanel = new ValuePanel("Horizontal Resolution:", 200);
		verResPanel = new ValuePanel("Vertical Resolution:", 200);
		
		startX = new ValuePanel("Start X:", 0);
		startY = new ValuePanel("Start Y:", 0);
		startZ = new ValuePanel("Start Z:", 0);
		startHeading = new ValuePanel("Start Heading:", 0);
		startPitch = new ValuePanel("Start Pitch:", 0);
		startRoll = new ValuePanel("Start Roll:", 0);
		
		horFOVPanel.setEditable(false);
		verFOVPanel.setEditable(false);
		horResPanel.setEditable(false);
		verResPanel.setEditable(false);
		
		degreesBox = new JCheckBox("Angles in Degrees");
		degreesBox.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if(degreesBox.isSelected()){
					maxAOAPanel.setValue((float)Math.toDegrees(maxAOAPanel.getValue()));
					startHeading.setValue((float)Math.toDegrees(startHeading.getValue()));
					startPitch.setValue((float)Math.toDegrees(startPitch.getValue()));
					startRoll.setValue((float)Math.toDegrees(startRoll.getValue()));
					horFOVPanel.setValue((float)Math.toDegrees(horFOVPanel.getValue()));
					verFOVPanel.setValue((float)Math.toDegrees(verFOVPanel.getValue()));
				} else {
					maxAOAPanel.setValue((float)Math.toRadians(maxAOAPanel.getValue()));
					startHeading.setValue((float)Math.toRadians(startHeading.getValue()));
					startPitch.setValue((float)Math.toRadians(startPitch.getValue()));
					startRoll.setValue((float)Math.toRadians(startRoll.getValue()));
					horFOVPanel.setValue((float)Math.toRadians(horFOVPanel.getValue()));
					verFOVPanel.setValue((float)Math.toRadians(verFOVPanel.getValue()));
				}
			}
		});
		
		confirmButton = new JButton("Load configurations to testbed");
		confirmButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				settingsListener.setConfigs(getConfig());
				settingsListener.setStartSettigs(getStartSettings());
			}
		});
		
		setLayout(new GridBagLayout());
		
		GridBagConstraints gc = new GridBagConstraints();
		
		gc.gridy = 0;
		gc.anchor = GridBagConstraints.CENTER;
		gc.weighty = 1;
		add(confirmButton,gc);
		
		gc.gridy++;
		gc.weighty = 0;
		gc.anchor = GridBagConstraints.LINE_END;
		add(degreesBox,gc);

		gc.gridy++;
		add(gravityPanel,gc);
		
		gc.gridy++;
		add(wingXPanel,gc);
		
		gc.gridy++;		
		add(tailSizePanel,gc);
		
		gc.gridy++;		
		add(engineMassPanel,gc);
		
		gc.gridy++;		
		add(wingMassPanel,gc);
		
		gc.gridy++;		
		add(tailMassPanel,gc);
		
		gc.gridy++;		
		add(maxThrustPanel,gc);
		
		gc.gridy++;		
		add(maxAOAPanel,gc);
		
		gc.gridy++;		
		add(wingSlopePanel,gc);
		
		gc.gridy++;	
		add(horStabSlopePanel,gc);
		
		gc.gridy++;	
		add(verStabSlopePanel,gc);
		
		gc.gridy++;	
		add(horFOVPanel,gc);
		
		gc.gridy++;	
		add(verFOVPanel,gc);
		
		gc.gridy++;	
		add(horResPanel,gc);
		
		gc.gridy++;	
		add(verResPanel,gc);
		
		gc.gridy++;	
		gc.weighty = 1;
		gc.anchor = GridBagConstraints.LAST_LINE_END;
		add(startX,gc);
		
		gc.gridy++;	
		gc.weighty = 0;
		add(startY,gc);
		
		gc.gridy++;	
		add(startZ,gc);
		
		gc.gridy++;	
		add(startHeading,gc);
		
		gc.gridy++;	
		add(startPitch,gc);
		
		gc.gridy++;	
		add(startRoll,gc);
	}
	
	public void addActionListener(ActionListener listener) {
		confirmButton.addActionListener(listener);
	}
	
	public float getGravity() {
		return gravityPanel.getValue();
	}
	
	public float getWingX() {
		return wingXPanel.getValue();
	}
	
	public float getTailSize() {
		return tailSizePanel.getValue();
	}
	
	public float getEngineMass() {
		return engineMassPanel.getValue();
	}
	
	public float getWingMass() {
		return wingMassPanel.getValue();
	}
	
	public float getTailMass() {
		return tailMassPanel.getValue();
	}
	
	public float getMaxThrust() {
		return maxThrustPanel.getValue();
	}
	
	public float getMaxAOA() {
		float value = maxAOAPanel.getValue();
		if(degreesBox.isSelected())
			return (float) Math.toRadians(value);
		else {
			return value;
		}
	}
	
	public float getWingSlope() {
		return wingSlopePanel.getValue();
	}
	
	public float getHorStabSlope() {
		return horStabSlopePanel.getValue();
	}
	
	public float getVerStabSlope() {
		return verStabSlopePanel.getValue();
	}
	
	public float getHorFOV() {
		float value = horFOVPanel.getValue();
		if(degreesBox.isSelected())
			return (float) Math.toRadians(value);
		else {
			return value;
		}
	}
	
	public float getVerFOV() {
		float value = verFOVPanel.getValue();
		if(degreesBox.isSelected())
			return (float) Math.toRadians(value);
		else {
			return value;
		}
	}
	
	public float getHorRes() {
		return horResPanel.getValue();
	}
	
	public float getVerRes() {
		return verResPanel.getValue();
	}
	
	public float getStartX(){
		return startX.getValue();
	}

	public float getStartY(){
		return startY.getValue();
	}
	
	public float getStartZ(){
		return startZ.getValue();
	}
	
	public float getStartHeading(){
		float value = startHeading.getValue();
		if(degreesBox.isSelected())
			return (float) Math.toRadians(value);
		else {
			return value;
		}
	}
	
	public float getStartPitch(){
		float value = startPitch.getValue();
		if(degreesBox.isSelected())
			return (float) Math.toRadians(value);
		else {
			return value;
		}
	}
	
	public float getStartRoll(){
		float value = startRoll.getValue();
		if(degreesBox.isSelected())
			return (float) Math.toRadians(value);
		else {
			return value;
		}
	}
	
	public AutopilotConfig getConfig() {
		float engineMass = ConfigPanel.this.getEngineMass();
		float gravity = ConfigPanel.this.getGravity();
		float horStab = ConfigPanel.this.getHorStabSlope();
		float verStab = ConfigPanel.this.getVerStabSlope();
		float horAOV = ConfigPanel.this.getHorFOV();
		float verAOV = ConfigPanel.this.getVerFOV();
		float maxAOA = ConfigPanel.this.getMaxAOA();
		float maxThrust = ConfigPanel.this.getMaxThrust();
		int cols = (int) ConfigPanel.this.getHorRes();
		int rows = (int) ConfigPanel.this.getVerRes();
		float tailMass = ConfigPanel.this.getTailMass();
		float tailSize = ConfigPanel.this.getTailSize();
		float wingLift = ConfigPanel.this.getWingSlope();
		float wingMass = ConfigPanel.this.getWingMass();
		float wingX = ConfigPanel.this.getWingX();
		return new AutopilotConfig() {
			public float getEngineMass() {
				return engineMass;
			}
			public float getGravity() {
				return gravity;
			}
			public float getHorStabLiftSlope() {
				return horStab;
			}
			public float getHorizontalAngleOfView() {
				return horAOV;
			}
			public float getMaxAOA() {
				return maxAOA;
			}
			public float getMaxThrust() {
				return maxThrust;
			}
			public int getNbColumns() {
				return cols;
			}
			public int getNbRows() {
				return rows;
			}
			public float getTailMass() {
				return tailMass;
			}
			public float getTailSize() {
				return tailSize;
			}
			public float getVerStabLiftSlope() {
				return verStab;
			}
			public float getVerticalAngleOfView() {
				return verAOV;
			}
			public float getWingLiftSlope() {
				return wingLift;
			}
			public float getWingMass() {
				return wingMass;
			}
			public float getWingX() {
				return wingX;
			}
			@Override
			public String getDroneID() {
				// TODO Auto-generated method stub
				return null;
			}
			@Override
			public float getWheelY() {
				return wheelY;
			}
			@Override
			public float getFrontWheelZ() {
				return frontZ;
			}
			@Override
			public float getRearWheelZ() {
				return rearZ;
			}
			@Override
			public float getRearWheelX() {
				return rearX;
			}
			@Override
			public float getTyreSlope() {
				return tyreSlope;
			}
			@Override
			public float getDampSlope() {
				return dampSlope;
			}
			@Override
			public float getTyreRadius() {
				return tyreRadius;
			}
			@Override
			public float getRMax() {
				return RMax;
			}
			@Override
			public float getFcMax() {
				return FcMax;
			}	
		};
	}
	
	public void setSettingsListener(SettingsListener listener) {
		this.settingsListener = listener;
	}
	
	public DroneStartSettings getStartSettings(){
		DroneStartSettings settings = new DroneStartSettings();
		settings.setPosition(new Vector3f(getStartX(),getStartY(),getStartZ()));
		settings.setHeading(getStartHeading());
		settings.setPitch(getStartPitch());
		settings.setRoll(getStartRoll());
		return settings;
	}
	
	public void setOtherDroneSettings(float wheelY, float frontZ, float rearZ,
			float rearX, float tyreSlope, float dampSlope, float tyreRadius, 
			float RMax, float FcMax) {
		this.wheelY = wheelY;
		this.frontZ = frontZ;
		this.rearZ = rearZ;
		this.rearX = rearX;
		this.tyreSlope = tyreSlope;
		this.dampSlope = dampSlope;
		this.tyreRadius = tyreRadius;
		this.RMax = RMax;
		this.FcMax = FcMax;
	}
}
