package swing_components;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Dimension;

import javax.swing.JPanel;

import org.lwjgl.LWJGLException;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;

public class CanvasPanel extends JPanel {
	
	private Canvas canvas;
	
	public CanvasPanel(int width, int height) {
		super();
		setLayout(new BorderLayout());
		canvas = new Canvas();
	    canvas.setSize(new Dimension(width, height));
        canvas.setMinimumSize(new Dimension(200, 200));
	    canvas.setIgnoreRepaint(true);
	    try {
		    Display.setDisplayMode(new DisplayMode(canvas.getWidth(), canvas.getHeight()));
		    Display.setResizable(true);
			Display.setParent(canvas);
		} catch (LWJGLException e1) {
			e1.printStackTrace();
		}

		super.add(canvas, BorderLayout.CENTER);
	}
}
