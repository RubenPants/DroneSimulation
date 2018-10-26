package swing_components;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import javax.swing.table.AbstractTableModel;

import org.lwjgl.util.vector.Vector3f;

import entities.Cube;

public class CubeTableModel extends AbstractTableModel{

	private List<Cube> cubes = new ArrayList<>();

	private final String[] headers = {"Number", "X", "Y", "Z", "Color"};
	
	public void setCubes(List<Cube> cubes) {
		this.cubes = cubes;
	}
	
	@Override
	public String getColumnName(int col) {
		return headers[col];
	}

	@Override
	public int getColumnCount() {
		return 5;
	}

	@Override
	public int getRowCount() {
		return cubes.size();
	}

	@Override
	public Object getValueAt(int row, int col) {
		Cube cube = cubes.get(row);
		
		switch(col) {
		case 0:
			return row;
		case 1:
			return cube.getPosition().x;
		case 2:
			return cube.getPosition().y;
		case 3: 
			return cube.getPosition().z;
		case 4:
			return Color.getHSBColor(cube.getHue(), cube.getSaturation(), 1);
		}
		return null;
	}
	
	@Override
	public void setValueAt(Object value, int row, int column) {
		Cube cube = cubes.get(row);
		if(column==4) {
			Color color = (Color)value;
			float[] hsb = Color.RGBtoHSB(color.getRed(), color.getGreen(), color.getBlue(), null);
			cube.setColor(hsb[0], hsb[1]);
		} else {
			float floatValue;
			try {
				floatValue = Float.parseFloat((String) value);
			} catch (NumberFormatException e) {
				floatValue = 0;
			}
			Vector3f oldPosition = cube.getPosition();
			if(column == 1)
				cube.setPosition(new Vector3f((float)floatValue,oldPosition.y,oldPosition.z));
			else if(column == 2)
				cube.setPosition(new Vector3f(oldPosition.x,(float)floatValue,oldPosition.z));
			else 
				cube.setPosition(new Vector3f(oldPosition.x,oldPosition.y,(float)floatValue));	
		}
	}

	@Override
	public boolean isCellEditable(int row, int column) {
    	return (column != 0);
	}

}