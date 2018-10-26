package tools;

import java.io.File;

import javax.swing.filechooser.FileFilter;

public class PathFileFilter extends FileFilter {

	@Override
	public boolean accept(File f) {
		if(f.isDirectory()){
			return true;
		}
		
		String name = f.getName();
		String extension = getFileExtension(name);
		
		if(extension == null) {
			return false;
		}
		if(extension.equals("txt")) {
			return true;
		}
		return false;
	}

	@Override
	public String getDescription() {
		return "Path file (*.txt)";
	}

	public static String getFileExtension(String name){
		int pointIndex = name.lastIndexOf(".");
		
		if(pointIndex == -1) {
			return null;
		}
		
		if(pointIndex == name.length() -1) {
			return null;
		}
		
		return name.substring(pointIndex+1, name.length());
	}
	
}
