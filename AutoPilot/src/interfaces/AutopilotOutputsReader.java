package interfaces;

/**
 * A class of autopilot outputs readers, to read autopilot outputs from a stream.
 * 
 * @author	Team Saffier
 * @version	1.0
 */
public class AutopilotOutputsReader {
	
	/**
	 * Read bytes from an input stream and store them in a buffer array.
	 * 
	 * @param 	stream
	 * 			The input stream to read from.
	 * @return	A buffer with the contents of the given input stream.
	 * @throws 	IOException
	 * 			If an I/O error occurs.
	 */
    @SuppressWarnings("unused")
	private static byte[] readByteArray(java.io.DataInputStream stream) throws java.io.IOException {
        int length = stream.readInt();
        byte[] array = new byte[length];
        stream.readFully(array);
        return array;
    }
    
    /**
     * Read bytes from an input stream and convert them to autopilot outputs.
     * 
     * @param 	stream
     * 			The input stream to read from.
     * @return	An autopilot outputs object having values corresponding to the given input stream.
     * @throws 	IOException
     * 			If an I/O error occurs.
     */
    public static AutopilotOutputs read(java.io.DataInputStream stream) throws java.io.IOException {
    	
    		// Read values
        float thrust = stream.readFloat();
        float leftWingInclination = stream.readFloat();
        float rightWingInclination = stream.readFloat();
        float horStabInclination = stream.readFloat();
        float verStabInclination = stream.readFloat();
        
        // Create autopilot outputs object
        return new AutopilotOutputs() {
            public float getThrust() { return thrust; }
            public float getLeftWingInclination() { return leftWingInclination; }
            public float getRightWingInclination() { return rightWingInclination; }
            public float getHorStabInclination() { return horStabInclination; }
            public float getVerStabInclination() { return verStabInclination; }
        };

    }
    
}