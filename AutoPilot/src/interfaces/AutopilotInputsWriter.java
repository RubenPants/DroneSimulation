package interfaces;

/**
 * A class of autopilot inputs writers, to write autopilot inputs to an output stream.
 * 
 * @author	Team Saffier
 * @version	1.0
 */
public class AutopilotInputsWriter {
	
	/**
	 * Write the given byte array to the given output stream.
	 * 
	 * @param 	stream
	 * 			The stream to write to.
	 * @param 	array
	 * 			The buffer to write to the output stream.
	 * @throws 	IOException
	 * 			If an I/O error occurs.
	 */
    private static void writeByteArray(java.io.DataOutputStream stream, byte[] array) throws java.io.IOException {
        stream.writeInt(array.length);
        stream.write(array);
    }
    
    /**
     * Write the given autopilot inputs to the given output stream.
     * 
     * @param 	stream
	 * 			The stream to write to.
	 * @param 	value
	 * 			The autopilot inputs object to write to the output stream.
	 * @throws 	IOException
	 * 			If an I/O error occurs.
     */
    public static void write(java.io.DataOutputStream stream, AutopilotInputs value) throws java.io.IOException {
        writeByteArray(stream, value.getImage());
        stream.writeFloat(value.getX());
        stream.writeFloat(value.getY());
        stream.writeFloat(value.getZ());
        stream.writeFloat(value.getHeading());
        stream.writeFloat(value.getPitch());
        stream.writeFloat(value.getRoll());
        stream.writeFloat(value.getElapsedTime());
    }
    
}