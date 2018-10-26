package swing_components;

import java.util.ArrayList;
import java.util.List;

import javax.swing.table.AbstractTableModel;

import entities.Cube;
import worldSimulation.Path;

public class PathsTableModel extends AbstractTableModel{

	private List<Path> paths = new ArrayList<>();

	private final String[] headers = {"Path number"};
	
	public void setPaths(List<Path> paths) {
		this.paths = paths;
	}
	
	public List<Cube> getPathCubes(int row){
		return paths.get(row).getCubes();
	}
	
	@Override
	public String getColumnName(int col) {
		return headers[col];
	}

	@Override
	public int getColumnCount() {
		return 1;
	}

	@Override
	public int getRowCount() {
		return paths.size();
	}

	@Override
	public Object getValueAt(int row, int col) {
		String name = paths.get(row).getName();
		if (name == null)
			return "Path " + row;
		else 
			return name;
	}

	@Override
	public boolean isCellEditable(int rowIndex, int columnIndex) {
		return true;
	}

	@Override
	public void setValueAt(Object aValue, int rowIndex, int columnIndex) {
		paths.get(rowIndex).setName(aValue.toString());
	}
}
