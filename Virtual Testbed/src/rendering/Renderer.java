package rendering;

import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL13;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;
import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import entities.Camera;
import entities.Cube;
import entities.Drone;
import entities.Entity;
import shaders.StaticShader;
import shaders.TextureShader;
import worldSimulation.World;

public class Renderer {
	
	private float RED = 1;
	private float GREEN = 1;
	private float BLUE = 1;
	
	private float FOV = 120;
	private float NEAR_PLANE = 0.1f;
	private float FAR_PLANE = 2000f;
	
	private StaticShader shader;
	private TextureShader texShader;
	private Matrix4f projectionMatrix;
	
	
	public Renderer(){
		shader = new StaticShader();
		texShader = new TextureShader();
		GL11.glEnable(GL11.GL_CULL_FACE);
		GL11.glCullFace(GL11.GL_BACK);
		calculatePerspectiveMatrix();
	}
	
	
	public void renderCubes(World world, Camera camera, float scale){
		prepareRender();
		
		shader.start();
		shader.loadViewMatrix(camera);
		for(Cube cube: world.getCubes()) {
			shader.loadTransformationMatrix(createTransformationMatrix(cube.getPosition(), cube.getHeading(), cube.getPitch(), cube.getRoll(), scale));
			shader.loadProjectionMatrix(projectionMatrix);
			shader.loadHueSaturation(cube.getHue(), cube.getSaturation());
			shader.loadRGB(cube.getRGB());
			GL30.glBindVertexArray(cube.getModel().getVaoID());
			GL20.glEnableVertexAttribArray(0); 
			GL20.glEnableVertexAttribArray(1); 
	
			GL11.glDrawElements(GL11.GL_TRIANGLES, cube.getModel().getVertexCount(), GL11.GL_UNSIGNED_INT, 0);
			
			GL20.glDisableVertexAttribArray(0);
			GL20.glDisableVertexAttribArray(1);
			GL30.glBindVertexArray(0);
		}
		shader.stop();
	}
	
	public void renderTex(Drone drone, Camera camera){
		Entity entity = drone.getEntity();
		GL11.glDisable(GL11.GL_CULL_FACE);
		texShader.start();
		texShader.loadViewMatrix(camera);
		texShader.loadTransformationMatrix(createTransformationMatrix(entity.getPosition(), entity.getHeading(), entity.getPitch(), entity.getRoll(), entity.getScale()));
		texShader.loadProjectionMatrix(projectionMatrix);
		GL30.glBindVertexArray(entity.getModel().getVaoID());
		GL20.glEnableVertexAttribArray(0); 
		GL20.glEnableVertexAttribArray(1); 
		GL13.glActiveTexture(GL13.GL_TEXTURE0);
		GL11.glBindTexture(GL11.GL_TEXTURE_2D, drone.getTextureID());

		GL11.glDrawElements(GL11.GL_TRIANGLES, entity.getModel().getVertexCount(), GL11.GL_UNSIGNED_INT, 0);
		
		GL20.glDisableVertexAttribArray(0);
		GL20.glDisableVertexAttribArray(1);
		GL30.glBindVertexArray(0);
		texShader.stop();
		GL11.glEnable(GL11.GL_CULL_FACE);
	}
	
	public void renderTex(Drone drone, Camera camera, float scale){
		Entity entity = drone.getEntity();
		GL11.glDisable(GL11.GL_CULL_FACE);
		texShader.start();
		texShader.loadViewMatrix(camera);
		texShader.loadTransformationMatrix(createTransformationMatrix(entity.getPosition(), entity.getHeading(), entity.getPitch(), entity.getRoll(), scale));
		texShader.loadProjectionMatrix(projectionMatrix);
		GL30.glBindVertexArray(entity.getModel().getVaoID());
		GL20.glEnableVertexAttribArray(0); 
		GL20.glEnableVertexAttribArray(1); 
		GL13.glActiveTexture(GL13.GL_TEXTURE0);
		GL11.glBindTexture(GL11.GL_TEXTURE_2D, drone.getTextureID());

		GL11.glDrawElements(GL11.GL_TRIANGLES, entity.getModel().getVertexCount(), GL11.GL_UNSIGNED_INT, 0);
		
		GL20.glDisableVertexAttribArray(0);
		GL20.glDisableVertexAttribArray(1);
		GL30.glBindVertexArray(0);
		texShader.stop();
		GL11.glEnable(GL11.GL_CULL_FACE);
	}
	
	private void prepareRender(){
		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glClearColor(RED, GREEN, BLUE, 1);
		GL11.glClear(GL11.GL_COLOR_BUFFER_BIT|GL11.GL_DEPTH_BUFFER_BIT); // | = OR operator
		
	}
	
	public void cleanUp(){
		shader.cleanUp();
	}
	
	public void calculatePerspectiveMatrix(){
        float aspectRatio = (float) Display.getWidth() / (float) Display.getHeight();
        float y_scale = (float) ((1f / Math.tan(Math.toRadians(FOV/2f))) * aspectRatio);
        float x_scale = y_scale / aspectRatio;
        float frustum_length = FAR_PLANE - NEAR_PLANE;
        
        projectionMatrix = new Matrix4f();
        projectionMatrix.m00 = x_scale;
        projectionMatrix.m11 = y_scale;
        projectionMatrix.m22 = -((FAR_PLANE + NEAR_PLANE) / frustum_length);
        projectionMatrix.m23 = -1;
        projectionMatrix.m32 = -((2 * NEAR_PLANE * FAR_PLANE) / frustum_length);
        projectionMatrix.m33 = 0;
	}
	
	public Matrix4f calculateOrthogonalMatrix(float width, float height) {
		float r = width/2;
		float t = height/2;
        float frustum_length = FAR_PLANE - NEAR_PLANE;
		
		projectionMatrix = new Matrix4f();
		projectionMatrix.m00 = 1/r;
		projectionMatrix.m11 = 1/t;
		projectionMatrix.m22 = -2/frustum_length;
		projectionMatrix.m32 = -((FAR_PLANE + NEAR_PLANE) / frustum_length);
		projectionMatrix.m33 = 1;
		
		return projectionMatrix;
	}
	
	public static Matrix4f createTransformationMatrix(Vector3f translation) {
		Matrix4f matrix = new Matrix4f();
		matrix.setIdentity();
		Matrix4f.translate(translation, matrix, matrix);
		return matrix;
	}
	
	public static Matrix4f createTransformationMatrix(Vector3f translation, 
			float heading, float pitch, float roll, float scale) {
		Matrix4f matrix = new Matrix4f();
		matrix.setIdentity();
		Matrix4f.translate(translation, matrix, matrix);
		Matrix4f.rotate((float) heading, new Vector3f(0,1,0), matrix, matrix);
		Matrix4f.rotate((float) pitch, new Vector3f(1,0,0), matrix, matrix);
		Matrix4f.rotate((float) roll, new Vector3f(0,0,1), matrix, matrix);
		Matrix4f.scale(new Vector3f(scale,scale,scale), matrix, matrix);
		return matrix;
	}
	
	public void changeSettings(float FOV, float f, float g, float h){
		this.FOV = FOV;
		this.RED = f;
		this.GREEN = g;
		this.BLUE = h;
		calculatePerspectiveMatrix();
	}
	
	public void setOrthogonalProjection() {
		Matrix4f mat = new Matrix4f();
		mat.setIdentity();
		shader.start();
		shader.loadProjectionMatrix(mat);
		shader.stop();
		texShader.start();
		texShader.loadProjectionMatrix(mat);
		texShader.stop();
	}
}
