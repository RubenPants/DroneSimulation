package swing_components;

import java.awt.FlowLayout;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

public class ValuePanel extends JPanel{
	
	private JLabel label;
	private JTextField field;
	private float value;
	
	
	public ValuePanel(String name, float defaultValue) {
		setLayout(new FlowLayout());
		
		value = defaultValue;
		label = new JLabel(name);
		field = new JTextField(Float.toString(defaultValue), 10);
		field.addKeyListener(new KeyAdapter(){
		      @Override
		      public void keyReleased(KeyEvent ke) {
		    	  String typed = field.getText();
		          try {
		        	  value = Float.parseFloat(typed);
		          } catch (NumberFormatException e) {
		        	  value = 0;
		          }
		      }
		});
		
		add(label);
		add(field);
	}
	
	public float getValue() {
		return value;
		
	}
	
	public void setEditable(boolean editable) {
		field.setEditable(editable);
	}
	
	public void setValue(float value){
		field.setText(Float.toString(value));
		this.value = value;
	}
}
