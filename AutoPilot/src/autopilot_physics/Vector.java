package autopilot_physics;

public class Vector {

	public Vector(float x, float y, float z) {
		setX(x);
		setY(y);
		setZ(z);
	}
	
	public Vector() {
		this(0,0,0);
	}
	
	private float x;
	
	private float y;
	
	private float z;
	
	public float getX() {
		return this.x;
	}
	
	public float getY() {
		return this.y;
	}
	
	public float getZ() {
		return this.z;
	}
	
	public void setX(float x) {
		this.x = x;
	}
	
	public void setY(float y) {
		this.y = y;
	}
	
	public void setZ(float z) {
		this.z = z;
		}
}
