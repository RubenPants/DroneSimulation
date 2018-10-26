package autopilot_physics;

import static java.lang.Math.toRadians;

import java.awt.Color;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;
import org.lwjgl.util.vector.Vector4f;

public class Tools {
	
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
}
