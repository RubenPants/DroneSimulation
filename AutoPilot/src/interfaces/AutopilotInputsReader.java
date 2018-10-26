package interfaces;

/**
 * A class of autopilot inputs readers, to read autopilot inputs from a stream.
 * 
 * @author	Team Saffier
 * @version	1.0
 */
public class AutopilotInputsReader {
	
	/**
	 * Read bytes from an input stream and store them in a buffer array.
	 * 
	 * @param 	stream
	 * 			The input stream to read from.
	 * @return	A buffer with the contents of the given input stream.
	 * @throws 	IOException
	 * 			If an I/O error occurs.
	 */
    private static byte[] readByteArray(java.io.DataInputStream stream) throws java.io.IOException {
        int length = stream.readInt();
        byte[] array = new byte[length];
        stream.readFully(array);
        return array;
    }
    
    /**
     * Read bytes from an input stream and convert them to autopilot inputs.
     * 
     * @param 	stream
     * 			The input stream to read from.
     * @return	An autopilot inputs object having values corresponding to the given input stream.
     * @throws 	IOException
     * 			If an I/O error occurs.
     */
    public static AutopilotInputs read(java.io.DataInputStream stream) throws java.io.IOException {
    	
    		// Read values
        byte[] image = readByteArray(stream);
        float x = stream.readFloat(), y = stream.readFloat(), z = stream.readFloat();
        float heading = stream.readFloat(), pitch = stream.readFloat(), roll = stream.readFloat();
        float elapsedTime = stream.readFloat();
        
        // Create autopilot inputs object
        return new AutopilotInputs() {
            public byte[] getImage() { return image; }
            public float getX() { return x; }
            public float getY() { return y; }
            public float getZ() { return z; }
            public float getHeading() { return heading; }
            public float getPitch() { return pitch; }
            public float getRoll() { return roll; }
            public float getElapsedTime() { return elapsedTime; }
        };

    }
    
}