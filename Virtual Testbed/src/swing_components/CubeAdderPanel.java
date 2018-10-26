package swing_components;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import org.lwjgl.util.vector.Vector3f;

public class CubeAdderPanel extends JPanel {

	private JLabel label;
	private JTextField fieldX;
	private JTextField fieldY;
	private JTextField fieldZ;
	private JButton btn;
	
	private SettingsListener listener;
	
	public CubeAdderPanel(){
		setLayout(new FlowLayout());
		
		label = new JLabel("Add target at position: ");
		fieldX = new JTextField(4);
		fieldY = new JTextField(4);
		fieldZ = new JTextField(4);
		btn = new JButton("Add");
		btn.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				if (listener != null){
					listener.addCube(getPosition());
				}
			}
		});
		
		add(label);
		add(fieldX);
		add(fieldY);
		add(fieldZ);
		add(btn);
	}
	
	public Vector3f getPosition(){
		return new Vector3f(getFieldValue(fieldX),getFieldValue(fieldY),getFieldValue(fieldZ));
	}
	
	public float getFieldValue(JTextField field){
        String typed = field.getText();
        float value;
		try {
			value = Float.parseFloat(typed);
		} catch (NumberFormatException e) {
			value = 0;
		}
        return value;
	}
	
	public void addSettingsListener(SettingsListener listener){
		this.listener = listener;
	}
}
