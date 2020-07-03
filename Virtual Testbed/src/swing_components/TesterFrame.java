package swing_components;

import java.awt.Color;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;

public class TesterFrame extends JFrame{
	
	private int testAmount;
	private int speed;
	private float longestTime;
	
	private float averageTime = 0;
	private int completedTestNumber = 0;
	private int testsDone = 0;
	
	private JLabel averageTimeLabel = new JLabel("0 seconds");
	private JLabel testsCompletedLabel = new JLabel("0 / 0");
	
	private JTextArea outputField;
	private JButton startBtn;
	private JButton pauseBtn;
	private JButton stopBtn;
	private JButton clearBtn;
	
	
	public TesterFrame(int testAmount, int speed, float timeToBeat){
		super("Run Tests");
		this.testAmount = testAmount;
		this.speed = speed;
		this.longestTime = timeToBeat;
		setSize(500,700);
		JPanel panel = new JPanel();
		panel.setLayout(new GridBagLayout());
		JLabel amountLabel = new JLabel("Amount of tests");
		amountLabel.setFont(new Font("Calibri", Font.PLAIN, 22));
		JLabel speedLabel = new JLabel("Relative speed");
		speedLabel.setFont(new Font("Calibri", Font.PLAIN, 22));
		JLabel longestTimeLabel = new JLabel("Time to beat");
		longestTimeLabel.setFont(new Font("Calibri", Font.PLAIN, 22));
		JTextField amountField = new JTextField(5);
		amountField.setText(Integer.toString(testAmount));
		amountField.addKeyListener(new KeyAdapter() {
			public void keyReleased(KeyEvent e) {
				try {
					TesterFrame.this.testAmount = Integer.parseInt(amountField.getText());
					amountField.setBackground(new Color(255,255,255));
				} catch(NumberFormatException ex) {
					amountField.setBackground(new Color(255, 165, 165));
				}
			}
		});
		JTextField speedField = new JTextField(5);
		speedField.setText(Integer.toString(speed));
		speedField.addKeyListener(new KeyAdapter() {
			public void keyReleased(KeyEvent e) {
				try {
					TesterFrame.this.speed = Integer.parseInt(speedField.getText());
					speedField.setBackground(new Color(255,255,255));
				} catch(NumberFormatException ex) {
					speedField.setBackground(new Color(255, 165, 165));
				}
			}
		});
		
		JTextField timeField = new JTextField(5);
		timeField.setText(Float.toString(timeToBeat));
		timeField.addKeyListener(new KeyAdapter() {
			public void keyReleased(KeyEvent e) {
				try {
					TesterFrame.this.longestTime = Float.parseFloat(timeField.getText());
					timeField.setBackground(new Color(255,255,255));
				} catch(NumberFormatException ex) {
					timeField.setBackground(new Color(255, 165, 165));
				}
			}
		});
		
		startBtn = new JButton("Start testing");
		pauseBtn = new JButton("Pause testing");
		pauseBtn.setEnabled(false);
		stopBtn = new JButton("Stop testing");
		clearBtn = new JButton("Reset testing");
		outputField = new JTextArea(5, 20);
		JScrollPane scrollPane = new JScrollPane(outputField); 
		outputField.setEditable(false);
		GridBagConstraints gc = new GridBagConstraints();
		
		/////////////////////////////////////////////////////////
		gc.gridx = 0;
		gc.gridy = 0;
		gc.weightx = .1;
		gc.weighty = .1;
		gc.anchor = GridBagConstraints.CENTER;
		panel.add(amountLabel, gc);
		/////////////////////////////////////////////////////////
		gc.gridx = 1;
		gc.gridy = 0;
		panel.add(amountField, gc);
		/////////////////////////////////////////////////////////
		gc.gridy++;
		gc.gridx = 0;
		panel.add(speedLabel, gc);
		/////////////////////////////////////////////////////////
		gc.gridx = 1;
		panel.add(speedField, gc);

		/////////////////////////////////////////////////////////
		gc.gridy++;
		gc.gridx = 0;
		panel.add(longestTimeLabel, gc);
		/////////////////////////////////////////////////////////
		gc.gridx = 1;
		panel.add(timeField, gc);

		/////////////////////////////////////////////////////////
		gc.gridy++;
		gc.gridx = 0;
		panel.add(startBtn, gc);

		/////////////////////////////////////////////////////////
		gc.gridx = 1;
		panel.add(clearBtn, gc);

		/////////////////////////////////////////////////////////
		gc.gridy++;
		gc.gridx = 0;
		panel.add(pauseBtn, gc);

		/////////////////////////////////////////////////////////
		gc.gridx = 1;
		panel.add(stopBtn, gc);
		
		/////////////////////////////////////////////////////////
		gc.gridy++;
		gc.gridx = 0;
		panel.add(new JLabel("Completed tests: "), gc);
		
		/////////////////////////////////////////////////////////
		gc.gridx = 1;
		panel.add(testsCompletedLabel, gc);

		/////////////////////////////////////////////////////////
		gc.gridy++;
		gc.gridx = 0;
		panel.add(new JLabel("Average Time: "), gc);

		/////////////////////////////////////////////////////////
		gc.gridx = 1;
		panel.add(averageTimeLabel, gc);
		/////////////////////////////////////////////////////////
		gc.gridy++;
		gc.gridx = 0;
		gc.gridwidth = 2;
		gc.weightx = 1;
		gc.weighty = 1;
		gc.fill = GridBagConstraints.BOTH;
		panel.add(scrollPane, gc);
		
		add(panel);
		
		setDefaultCloseOperation(HIDE_ON_CLOSE);
		setLocationRelativeTo(null);
	}
	
	public void addStartButtonListener(ActionListener listener) {
		startBtn.addActionListener(listener);
	}
	
	public void addStopButtonListener(ActionListener listener) {
		stopBtn.addActionListener(listener);
	}
	
	public void addPauseButtonListener(ActionListener listener) {
		pauseBtn.addActionListener(listener);
	}
	
	public void addClearButtonListener(ActionListener listener) {
		clearBtn.addActionListener(listener);
	}
	
	public void appendText(String text){
		outputField.append(text + "\n");
	}
	
	public int getAmountOfTests(){
		return testAmount;
	}
	
	public float getSpeed(){
		return speed;
	}
	
	public float getTimeToBeat(){
		return longestTime;
	}
	
	public void testCompleted(float time){
		completedTestNumber += 1;
		testsDone += 1;
		if(completedTestNumber==1){
			averageTime = time;
		} else {
			averageTime = averageTime*(completedTestNumber-1)/completedTestNumber + time/completedTestNumber;
		}
		averageTimeLabel.setText(averageTime + " seconds");
		testsCompletedLabel.setText(completedTestNumber + " / " + testsDone);
	}
	
	public void testFailed(String text){
		testsDone += 1;
		testsCompletedLabel.setText(completedTestNumber + " / " + testsDone);
		outputField.append(text + "\n");
	}
	
	public void reset(){
		testsDone = 0;
		completedTestNumber = 0;
		averageTime = 0;
		averageTimeLabel.setText("0 seconds");
		testsCompletedLabel.setText("0 / 0");
		outputField.setText("");
		startBtn.setEnabled(true);
	}
	
	public void lockTesting(boolean isTesting) {
		startBtn.setEnabled(!isTesting);
		pauseBtn.setEnabled(isTesting);
	}
	
	public void paused(boolean paused) {
		if(paused)
			pauseBtn.setText("Resume testing");
		else 
			pauseBtn.setText("Pause testing");
	}
}
