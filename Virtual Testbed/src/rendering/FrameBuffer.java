package rendering;

import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL30;

public class FrameBuffer {
	
    private int resolution_width;
    private int resolution_height;

    private int frameBufferID;
    private int renderRGBBufferID;
    private int renderDepthBufferID;
    
    public FrameBuffer(int res_width, int res_height) {
    	this.resolution_width = res_width;
    	this.resolution_height = res_height;
    	frameBufferID = createFrameBuffer();
    	renderRGBBufferID = createRenderRGBBufferAttachment(resolution_width, resolution_height);
    	renderDepthBufferID = createRenderDepthBufferAttachment(resolution_width, resolution_height);
    }
    
    private int createFrameBuffer() {
        int frameBuffer = GL30.glGenFramebuffers();
        GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, frameBuffer);
        GL11.glDrawBuffer(GL30.GL_COLOR_ATTACHMENT0);
        return frameBuffer;
    }
    
    public void bindFrameBuffer(){
        GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, frameBufferID);
        GL11.glViewport(0, 0, resolution_width, resolution_height);
    }
    
    public void unbindCurrentFrameBuffer() {
        GL30.glBindFramebuffer(GL30.GL_FRAMEBUFFER, 0);
        GL11.glViewport(0, 0, Display.getWidth(), Display.getHeight());
    }
    
    private int createRenderRGBBufferAttachment(int width, int height) {
        int renderBuffer = GL30.glGenRenderbuffers();
        GL30.glBindRenderbuffer(GL30.GL_RENDERBUFFER, renderBuffer);
        GL30.glRenderbufferStorage(GL30.GL_RENDERBUFFER, GL11.GL_RGB, width,
                height);
        GL30.glFramebufferRenderbuffer(GL30.GL_FRAMEBUFFER, GL30.GL_COLOR_ATTACHMENT0,
                GL30.GL_RENDERBUFFER, renderBuffer);
        return renderBuffer;
    }
    
    private int createRenderDepthBufferAttachment(int width, int height) {
        int renderBuffer = GL30.glGenRenderbuffers();
        GL30.glBindRenderbuffer(GL30.GL_RENDERBUFFER, renderBuffer);
        GL30.glRenderbufferStorage(GL30.GL_RENDERBUFFER, GL11.GL_DEPTH_COMPONENT, width,
                height);
        GL30.glFramebufferRenderbuffer(GL30.GL_FRAMEBUFFER, GL30.GL_DEPTH_ATTACHMENT,
                GL30.GL_RENDERBUFFER, renderBuffer);
        return renderBuffer;
    }
    
    
    public void cleanUp() {
        GL30.glDeleteFramebuffers(frameBufferID);
        GL30.glDeleteRenderbuffers(renderRGBBufferID);
        GL30.glDeleteRenderbuffers(renderDepthBufferID);    
    }
}
