package swing_components;

import java.awt.Color;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

import javax.security.auth.callback.ConfirmationCallback;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

public class AccuracyFrame extends JFrame{
	
	int autocalls;
	int iterations;
	int useConstantGaps;
	
	JButton confirmBtn;
	
	public AccuracyFrame(int fps, int iterations, boolean useCstTime){
		super("Testbed Accuracy Settings");
		this.autocalls = fps;
		this.iterations = iterations;
		setSize(450,180);
		JPanel panel = new JPanel();
		panel.setLayout(new GridBagLayout());
		JLabel autocallsLabel = new JLabel("Autopilot calls per second");
		autocallsLabel.setFont(new Font("Calibri", Font.PLAIN, 22));
		JLabel iterationsLabel = new JLabel("Newton iterations per call");
		iterationsLabel.setFont(new Font("Calibri", Font.PLAIN, 22));
		JTextField autocallsField = new JTextField(5);
		autocallsField.setText(Integer.toString(fps));
		autocallsField.addKeyListener(new KeyAdapter() {
			public void keyReleased(KeyEvent e) {
				try {
					AccuracyFrame.this.autocalls = Integer.parseInt(autocallsField.getText());
					autocallsField.setBackground(new Color(255,255,255));
				} catch(NumberFormatException ex) {
					autocallsField.setBackground(new Color(255, 165, 165));
				}
			}
		});
		JTextField iterationsField = new JTextField(5);
		iterationsField.setText(Integer.toString(iterations));
		iterationsField.addKeyListener(new KeyAdapter() {
			public void keyReleased(KeyEvent e) {
				try {
					AccuracyFrame.this.iterations = Integer.parseInt(iterationsField.getText());
					iterationsField.setBackground(new Color(255,255,255));
				} catch(NumberFormatException ex) {
					iterationsField.setBackground(new Color(255, 165, 165));
				}
			}
		});
		
		JCheckBox cstTimeBox = new JCheckBox("Use constant time gaps");
		cstTimeBox.setSelected(useCstTime);
		cstTimeBox.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				AccuracyFrame.this.useConstantGaps = cstTimeBox.isSelected() ? 1 : 0;
			}
		});
		confirmBtn = new JButton("Confirm");
		
		GridBagConstraints gc = new GridBagConstraints();
		
		/////////////////////////////////////////////////////////
		gc.gridx = 0;
		gc.gridy = 0;
		gc.weightx = 1;
		gc.weighty = 1;
		panel.add(autocallsLabel, gc);
		/////////////////////////////////////////////////////////
		gc.gridx = 1;
		gc.gridy = 0;
		panel.add(autocallsField, gc);
		/////////////////////////////////////////////////////////
		gc.gridx = 0;
		gc.gridy = 1;
		panel.add(iterationsLabel, gc);
		/////////////////////////////////////////////////////////
		gc.gridx = 1;
		gc.gridy = 1;
		panel.add(iterationsField, gc);

		/////////////////////////////////////////////////////////
		gc.gridx = 0;
		gc.gridy = 2;
		panel.add(cstTimeBox, gc);

		/////////////////////////////////////////////////////////
		gc.gridx = 1;
		gc.gridy = 2;
		panel.add(confirmBtn, gc);
		
		add(panel);
		
		setDefaultCloseOperation(DISPOSE_ON_CLOSE);
		setLocationRelativeTo(null);
		setVisible(true);
	}
	
	public int[] getSettings() {
		return new int[] {autocalls,iterations,useConstantGaps};
	}
	
	public void addButtonListener(ActionListener listener) {
		confirmBtn.addActionListener(listener);
	}
}
