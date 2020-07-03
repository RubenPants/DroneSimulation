package entities;

import org.lwjgl.util.vector.Vector2f;
import org.lwjgl.util.vector.Vector3f;

import rendering.Loader;

public class Airport {
	private float w;
	private float l;
	
	private Vector2f position;
	
	private int textureID;
	
	private Entity entity;
	
	private final float HEIGHT = 0.01f;
	
	public Airport(float gateW, float laneL, Vector2f position, int textureID, Loader loader){
		this.w = gateW;
		this.l = laneL;
		this.position = position;
		this.textureID = textureID;
		setEntity(loader);
	}
	
	private void setEntity(Loader loader){
		float[] positions = {-w/2, 0, -l/2,
							 -w/2, 0,  l/2,
							 w/2 , 0,  l/2,
							 w/2 , 0, -l/2};
		float[] textureCoords = { 0, 0,
								  1, 0,
								  1, 1,
								  0, 1};
		float[] normals = {0,1,0,0,1,0,0,1,0,0,1,0};
		int[] indices = {0,1,3,	3,1,2};
		Model model = loader.loadToVAO(positions, textureCoords, normals, indices);
		this.entity = new Entity(model, new Vector3f(position.x, HEIGHT, position.y));
	}
	
	public Entity getEntity(){
		return this.entity;
	}
	
	public int getTextureID(){
		return this.textureID;
	}
}
