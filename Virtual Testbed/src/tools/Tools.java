package tools;

import static java.lang.Math.toRadians;

import java.awt.Color;
import java.math.BigDecimal;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector2f;
import org.lwjgl.util.vector.Vector3f;
import org.lwjgl.util.vector.Vector4f;

import entities.Camera;

public class Tools {

	public static Vector3f HSVtoRGB(float hue, float saturation, float value) {
	    int rgb = Color.HSBtoRGB(hue, saturation, value);
	    float red = (float) (((rgb >> 16) & 0xFF)/255.0);
	    float green = (float) (((rgb >> 8) & 0xFF)/255.0);
	    float blue = (float) ((rgb & 0xFF)/255.0);
	    return new Vector3f(red, green, blue);
	}
	
	public static float[] getCubeVertices(float width,float height,float length){
		float[] result={
				-width/2,height/2,length/2,	
				-width/2,-height/2,length/2,	//front positieve Z 70
				width/2,-height/2,length/2,	
				width/2,height/2,length/2,		
				
				width/2,height/2,-length/2,	
				width/2,-height/2,-length/2,	//back negatieve Z 45
				-width/2,-height/2,-length/2,	
				-width/2,height/2,-length/2,
				
				-width/2,height/2,-length/2,	
				-width/2,-height/2,-length/2,	//left	negatieve X 30
				-width/2,-height/2,length/2,	
				-width/2,height/2,length/2,
				
				width/2,height/2,length/2,	
				width/2,-height/2,length/2,		//right	positieve X 85
				width/2,-height/2,-length/2,	
				width/2,height/2,-length/2,
				
				-width/2,height/2,-length/2,
				-width/2,height/2,length/2,		//top positieve Y 100
				width/2,height/2,length/2,
				width/2,height/2,-length/2,
				
				-width/2,-height/2,length/2,
				-width/2,-height/2,-length/2,	//bottom negatieve Y 15
				width/2,-height/2,-length/2,
				width/2,-height/2,length/2	
		};
		return result;
	}
	
	public static float[] calculateCubeColorValues() {
		float[] values = {.7f, .7f, .7f, .7f,
						  .45f, .45f, .45f, .45f,
						  .30f, .30f, .30f, .30f,
						  .85f, .85f, .85f, .85f,
						  1.00f, 1.00f, 1.00f, 1.00f,
						  .15f, .15f, .15f, .15f};
		return values;
	}
	
	public static Matrix4f createViewMatrix(Camera camera){
		  Matrix4f viewMatrix = new Matrix4f();
		  viewMatrix.setIdentity();
		  Matrix4f.rotate((float) -camera.getRoll(), new Vector3f(0,0,1), viewMatrix,
		    viewMatrix);
		  Matrix4f.rotate((float) -camera.getPitch(), new Vector3f(1,0,0), viewMatrix,
		    viewMatrix);
		  Matrix4f.rotate((float) -camera.getHeading(), new Vector3f(0,1,0), viewMatrix,
				    viewMatrix);
		  Vector3f cameraPos = camera.getPosition();
		  Vector3f negativeCameraPos = new Vector3f(-cameraPos.x,-cameraPos.y,-cameraPos.z);
		  Matrix4f.translate(negativeCameraPos, viewMatrix, viewMatrix);
		  return viewMatrix;
	}
	
	public static Matrix4f createViewMatrixNoTranslate(Camera camera){
		  Matrix4f viewMatrix = new Matrix4f();
		  viewMatrix.setIdentity();
		  Matrix4f.rotate((float) -camera.getRoll(), new Vector3f(0,0,1), viewMatrix,
		    viewMatrix);
		  Matrix4f.rotate((float) -camera.getPitch(), new Vector3f(1,0,0), viewMatrix,
		    viewMatrix);
		  Matrix4f.rotate((float) -camera.getHeading(), new Vector3f(0,1,0), viewMatrix,
				    viewMatrix);
		  return viewMatrix;
	}
	
	public static Vector3f transformVector(Matrix4f matrix, Vector3f vector) {
		Vector4f vector4 = new Vector4f(vector.x, vector.y, vector.z, 1);
		Matrix4f.transform(matrix, vector4, vector4);
		return new Vector3f(vector4.x, vector4.y, vector4.z);
	}

	public static Vector3f addVectors(Vector3f... vectors) {
		Vector3f result = new Vector3f();
		for(Vector3f vector: vectors) {
			Vector3f.add(result, vector, result);
		}
		return result;
	}
	
	public static Vector3f scaleVector(Vector3f vector, float scale) {
		return new Vector3f(scale*vector.x, scale*vector.y, scale*vector.z);
	}
	
	public static void printVector(String name, Vector3f vector) {
		System.out.println(name + " " + vector.x + " " + vector.y + " " + vector.z);
	}
	
    public static float round(double number, int decimalPlace) {
        BigDecimal bd = new BigDecimal(number);
        bd = bd.setScale(decimalPlace, BigDecimal.ROUND_HALF_UP);
        return bd.floatValue();
    }
    
	public static Matrix4f createTransformationMatrix(Vector2f translation, Vector2f scale) {
		Matrix4f matrix = new Matrix4f();
		matrix.setIdentity();
		Matrix4f.translate(translation, matrix, matrix);
		Matrix4f.scale(new Vector3f(scale.x, scale.y, 1f), matrix, matrix);
		return matrix;
	}
}
