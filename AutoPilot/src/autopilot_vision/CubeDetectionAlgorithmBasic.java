package autopilot_vision;

import java.awt.Color;
import java.util.ArrayList;

/**
 * A class of cube detection algorithms using basic logic.
 * 	The algorithms can scan a given 2-dimensional image for a cube of given color,
 * 	or scan such an image for any cube of any color.
 * 
 * @author	Team Saffier
 * @version 1.0
 */
public class CubeDetectionAlgorithmBasic extends CubeDetectionAlgorithm {
	
	@Override
	public ArrayList<Cube> locateUnitCubes(Image image) {
		
		// Pre-processing
		ArrayList<Cube> cubes = new ArrayList<Cube>();
		byte[] pixels = image.getPixels();
		int width = image.getSize().getWidth();
		int i = 0, length = pixels.length, row = 0, column = 0, lastRow, lastColumn;
		float[] currentHSB = new float[3];
		float[] lastHS = new float[2];
		boolean match = false;
		lastHS[0] = -1; // Denotes white color
		
		// Calculate bounding box
		while (i < length) {
						
			// Get the current hue/saturation values
			if (pixels[i] == -1 && pixels[i+1] == -1 && pixels[i+2] == -1)
				currentHSB[0] = currentHSB[1] = -1; // Denotes white color
			else
				Color.RGBtoHSB((pixels[i] & 0xff), (pixels[i+1] & 0xff), (pixels[i+2] & 0xff), currentHSB);
			
			// Track the transition from/to cube
			if ((currentHSB[0] != lastHS[0] && currentHSB[1] != lastHS[1]) // Color/cube transition (rather rare in most cases)
				|| column == 0) {
				
				// Last pixel was cube (can't be first pixel), update its bounding box
				if (lastHS[0] != -1) { 
					
					// Get row/column of previous pixel
					lastColumn = column - 1;
					if (lastColumn < 0) {
						lastColumn = width - 1;
						lastRow = row - 1; // Can't be negative or it would be the first pixel, where lastHS[0] would be -1
					}
					else
						lastRow = row;
					
					// Loop through cubes and update bounding box
					match = false;
					for (Cube cube : cubes) {
						float huediff = Math.abs(cube.getHue() - lastHS[0]);
						float satdiff = Math.abs(cube.getSaturation() - lastHS[1]);
						if ((huediff < THRESHOLD || 1.0 - huediff < THRESHOLD)
							&& (satdiff < THRESHOLD || 1.0 - satdiff < THRESHOLD)) {
							match = true;
							cube.getBoundingBox().spanPoint(lastColumn, lastRow);
							break;
						}
					}
					if (!match)
						cubes.add(new Cube(new Rectangle2D(column, column, row, row), lastHS[0], lastHS[1]));
					
				}
				
				// Current pixel is cube, update its bounding box
				if (currentHSB[0] != -1) { 
					
					// Loop through cubes and update bounding box
					match = false;
					for (Cube cube : cubes) {
						float huediff = Math.abs(cube.getHue() - currentHSB[0]);
						float satdiff = Math.abs(cube.getSaturation() - currentHSB[1]);
						if ((huediff < THRESHOLD || 1.0 - huediff < THRESHOLD)
							&& (satdiff < THRESHOLD || 1.0 - satdiff < THRESHOLD)) {
							match = true;
							cube.getBoundingBox().spanPoint(column, row);
							break;
						}
					}
					if (!match)
						cubes.add(new Cube(new Rectangle2D(column, column, row, row), currentHSB[0], currentHSB[1]));
					
				}
				
			}
			
			lastHS[0] = currentHSB[0];
			lastHS[1] = currentHSB[1];
			
			// Next pixel
			i += 3;
			if (++column == width) {
				column = 0;
				row++;
			}
			
		}
		
		return cubes;
		
	}
	
	@Override
	public Cube locateUnitCube(Image image, float hue, float saturation) {
		
		// Pre-processing
		byte[] pixels = image.getPixels();
		int minX = 0, maxX = 0, minY = 0, maxY = 0; // The bounding box
		int i = 0, length = pixels.length, row = 0, column = 0, width = image.getSize().getWidth();
		boolean noMatch = true, emptyRow = false;
		float[] currentHSB = new float[3];
						
		// Calculate bounding box
		while (i < length) {
			
			Color.RGBtoHSB((pixels[i] & 0xff), (pixels[i+1] & 0xff), (pixels[i+2] & 0xff), currentHSB);
			
			// Inspect current pixel
			if (Math.abs(currentHSB[0] - hue) < 0.1f && Math.abs(currentHSB[1] - saturation) < 0.1f) {
				// Color match (only the brightness/value may differ)
				// Now update the bounding box
				if (noMatch) {
					minX = maxX = column;
					minY = maxY = row;
					noMatch = false;
				}
				else {
					if (column < minX) minX = column;
					else if (column > maxX) maxX = column;
					if (row < minY) minY = row;
					else if (row > maxY) maxY = row;
				}
				emptyRow = false;
			}
			
			// Next pixel
			i += 3;
			if (++column == width) {
				if (emptyRow && !noMatch)
					break;
				column = 0;
				row++;
				emptyRow = true;
			}
			
		}
		
		// Nothing was detected
		if (noMatch) {
			return null;
		}
		
		// Calculate center
		Rectangle2D bounds = new Rectangle2D(minX, maxX, minY, maxY);
		return new Cube(bounds, hue, saturation);
		
	}
	
}