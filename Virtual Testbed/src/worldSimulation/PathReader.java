package worldSimulation;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import org.lwjgl.util.vector.Vector3f;

import entities.Cube;
import entities.Model;

/**
 * a world reader reads an  xml/ods file and creates a world out of it 
 * with the destined cubes as specified in the file
 * @author Wout
 *
 */
public class PathReader {
	
	public Path ReadAndCreatePathFromText(File file,Model createModel){
		Path path = new Path();
		FileReader isr = null;
		 try {
	            isr = new FileReader(file);
	        } catch (FileNotFoundException e) {
	            System.err.println("File not found  ; don't use any extention");
	        }
		BufferedReader reader = new BufferedReader(isr);
		String line;
		try {
			while(reader.ready()){
				line=reader.readLine();
				 String[] currentLine = line.split(" ");
				 float xposition=Float.parseFloat(currentLine[0]);
				 float yposition=Float.parseFloat(currentLine[1]);
				 float zposition=Float.parseFloat(currentLine[2]);
				 Cube cube= new Cube(createModel, new Vector3f(xposition,yposition,zposition));
				 path.addCube(cube);
				
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		path.setName(file.getName());
		return path;
	}
	
	public void writePathToTextFile(File file, Path path){
		BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter(file));
            for(Cube cube: path.getCubes()){
            	Vector3f pos = cube.getPosition();
            	writer.write(pos.x + " " + pos.y + " " + pos.z);
            	writer.newLine();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                writer.close();
            } catch (Exception e) {
            }
        }
	}

}