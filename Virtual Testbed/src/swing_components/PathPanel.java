package swing_components;

import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.TableColumn;

import org.lwjgl.util.vector.Vector3f;

import interfaces.AutopilotConfig;
import entities.Cube;
import worldSimulation.DroneStartSettings;
import worldSimulation.Path;

public class PathPanel extends JPanel{
	
	private JTable pathsTable;
	private PathsTableModel pathsModel;
	private JPopupMenu popupPathMenu;
	
	private JTable cubesTable;
	private CubeTableModel cubesModel;
	private JPopupMenu popupCubeMenu;
	
	private CubeAdderPanel adderPanel;
	
	private PathListener pathListener;
	
	public PathPanel() {
		pathsModel = new PathsTableModel();
		pathsTable = new JTable(pathsModel);
		pathsTable.getTableHeader().setReorderingAllowed(false);
		popupPathMenu = new JPopupMenu();
		JMenuItem removePathItem = new JMenuItem("Remove Path");
		removePathItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				int row = pathsTable.getSelectedRow();
				pathListener.pathDeleted(row);
				pathsModel.fireTableDataChanged();
				cubesModel.setCubes(new ArrayList<>());
				cubesModel.fireTableDataChanged();
			}
		});
		
		popupPathMenu.add(removePathItem);
		
		pathsTable.addMouseListener(new MouseAdapter() {
			@Override
			public void mousePressed(MouseEvent e) {
				int row = pathsTable.rowAtPoint(e.getPoint());
				
				pathsTable.getSelectionModel().setSelectionInterval(row, row);
				
				if(e.getButton() == MouseEvent.BUTTON1) {
					cubesModel.setCubes(pathsModel.getPathCubes(row));
					cubesModel.fireTableDataChanged();
				}
				if(e.getButton() == MouseEvent.BUTTON3) {
					popupPathMenu.show((Component) e.getSource(),e.getX(),e.getY());
				}
			}
		});
		
		JScrollPane scrollPane1 = new JScrollPane(pathsTable);
		
		scrollPane1.setPreferredSize(new Dimension(200,250));
		
		JButton createPathButton = new JButton("Create empty Path");
		createPathButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				pathListener.newPathCreated();
				pathsModel.fireTableDataChanged();
			}
		});
		
		JButton randomPathButton = new JButton("Generate random Path");
		randomPathButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				pathListener.generateRandomPath();
				pathsModel.fireTableDataChanged();
			}
		});
		
		cubesModel = new CubeTableModel();
		cubesTable = new JTable(cubesModel);
		cubesTable.getTableHeader().setReorderingAllowed(false);
		TableColumn colorColumn = cubesTable.getColumnModel().getColumn(4);
		colorColumn.setCellEditor(new ColorChooserEditor());

		popupCubeMenu = new JPopupMenu();
		JMenuItem removeCubeItem = new JMenuItem("Remove Cube");
		removeCubeItem.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if (cubesTable.isEditing())
				    cubesTable.getCellEditor().stopCellEditing();
				int rowPath = pathsTable.getSelectedRow();
				int rowCube = cubesTable.getSelectedRow();
				pathListener.cubeRemoved(rowCube, rowPath);
				cubesModel.fireTableDataChanged();
			}
		});
		popupCubeMenu.add(removeCubeItem);
		
		cubesTable.addMouseListener(new MouseAdapter() {
			@Override
			public void mousePressed(MouseEvent e) {
				int row = cubesTable.rowAtPoint(e.getPoint());
				cubesTable.getSelectionModel().setSelectionInterval(row, row);

				if(e.getButton() == MouseEvent.BUTTON3) {
					popupCubeMenu.show((Component) e.getSource(),e.getX(),e.getY());
				}
			}
		});
		
		cubesTable.setDefaultRenderer(Object.class, new DefaultTableCellRenderer() {
			@Override
		    public Component getTableCellRendererComponent(JTable table,
		            Object value, boolean isSelected, boolean hasFocus, int row, int col) {

		        Component c = super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, col);

		        Color color = (Color)table.getModel().getValueAt(row, 4);
		        if (! table.isRowSelected(row))
		        {
		            if(col == 4)
		                c.setBackground(color);
		            else
		                c.setBackground(table.getBackground());
		        }
		        return this;
		    }  
		});
		addMouseListener(new MouseAdapter() {
			public void mouseClicked(MouseEvent e) {
				cubesTable.getSelectionModel().clearSelection();
			}
		});
		JScrollPane scrollPane2 = new JScrollPane(cubesTable);
		
		scrollPane2.setPreferredSize(new Dimension(380,250));
		
		adderPanel = new CubeAdderPanel();
		adderPanel.addSettingsListener(new SettingsListener() {
			public void changeRenderSettings(SettingsEvent e) {}
			public void addCube(Vector3f position) {
				int row = pathsTable.getSelectedRow();
				if(row!=-1) {
					pathListener.cubeAdded(adderPanel.getPosition(),row);
					cubesModel.fireTableDataChanged();
				}
			}
			public void setTime(float time) {}
			public void setConfigs(AutopilotConfig config) {}
			public void setStartSettigs(DroneStartSettings settings) {}
		});
		
		JButton loadPathButton = new JButton("Load path to simulation");
		loadPathButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if (cubesTable.isEditing())
				    cubesTable.getCellEditor().stopCellEditing();
				int pathIndex = pathsTable.getSelectedRow();
				if(pathIndex!=-1)
					pathListener.loadPathToWorld(pathIndex);
			}
		});
		
		setLayout(new GridBagLayout());
		GridBagConstraints gc = new GridBagConstraints();
		
		/////////////////////////////////////////
		gc.gridx = 0;
		gc.gridy = 0;
		gc.anchor = GridBagConstraints.FIRST_LINE_START;
		add(new JLabel("Imported Paths"), gc);
		//////////////////////////////////////////
		
		gc.gridy++;
		gc.anchor = GridBagConstraints.CENTER;
		add(scrollPane1, gc);
		
		///////////////////////////////////////////
		gc.gridy++;
		gc.anchor = GridBagConstraints.CENTER;
		add(createPathButton, gc);

		///////////////////////////////////////////
		gc.gridy++;
		gc.anchor = GridBagConstraints.CENTER;
		add(randomPathButton, gc);
		
		/////////////////////////////////////////////
		gc.gridy++;
		gc.anchor = GridBagConstraints.FIRST_LINE_START;
		add(new JLabel("Cubes in the selected path above"), gc);
		
		//////////////////////////////////////////
		gc.gridy++;
		gc.anchor = GridBagConstraints.CENTER;
		add(scrollPane2, gc);
		
		//////////////////////////////////////////
		gc.gridy++;
		gc.anchor = GridBagConstraints.CENTER;
		add(adderPanel, gc);
		
		//////////////////////////////////////////
		gc.gridy++;
		gc.anchor = GridBagConstraints.CENTER;
		add(loadPathButton, gc);
	}
	
	public void addPathListener(PathListener listener) {
		this.pathListener = listener;
	}
	
	public void setPathData(List<Path> paths) {
		pathsModel.setPaths(paths);
	}
	
	public void setCubesData(List<Cube> cubes) {
		cubesModel.setCubes(cubes);
	}
	
	public void refreshTables() {
		pathsModel.fireTableDataChanged();
		cubesModel.fireTableDataChanged();
	}
}