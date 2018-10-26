package rendering;

import java.nio.ByteBuffer;

import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL30;
import org.lwjgl.opengl.GL32;

public class TextureFrameBuffer {
	
    private int resolution_width;
    private int resolution_height;

    private int frameBufferID;
    private int textureTopID;
    
    public TextureFrameBuffer(int res_width, int res_height) {
    	this.resolution_width = res_width;
    	this.resolution_height = res_height;
    	frameBufferID = createFrameBuffer();
    	textureTopID = createTextureAttachment(resolution_width, resolution_height);
    }
    
    public int getTexture() {
    	return textureTopID;
    }
    
    private int createFrameBuffer() {
        int frameBuffer = GL30.glGenFramebuffers();
        GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, frameBuffer);
        GL11.glDrawBuffer(GL30.GL_COLOR_ATTACHMENT0);
        return frameBuffer;
    }
    
    public void bindFrameBuffer(){
    	GL11.glBindTexture(GL11.GL_TEXTURE_2D, 0);
        GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, frameBufferID);
        GL11.glViewport(0, 0, resolution_width, resolution_height);
    }
    
    public void unbindCurrentFrameBuffer() {
    	GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, 0);
        GL11.glViewport(0, 0, Display.getWidth(), Display.getHeight());
    }
    
    private int createTextureAttachment( int width, int height) {
        int texture = GL11.glGenTextures();
        GL11.glBindTexture(GL11.GL_TEXTURE_2D, texture);
        GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGB, width, height,
                0, GL11.GL_RGB, GL11.GL_UNSIGNED_BYTE, (ByteBuffer) null);
        GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_LINEAR);
        GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER, GL11.GL_LINEAR);
        GL32.glFramebufferTexture(GL30.GL_FRAMEBUFFER, GL30.GL_COLOR_ATTACHMENT0,
                texture, 0);
        return texture;
    }
    
    
    public void cleanUp() {
        GL30.glDeleteFramebuffers(frameBufferID);
        GL11.glDeleteTextures(textureTopID); 
    }
}
