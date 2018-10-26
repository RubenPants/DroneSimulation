package entities;

public class TexturedModel extends Model{

	private int textureID;
	
	public TexturedModel(int vaoID, int vertexCount, int textureID) {
		super(vaoID, vertexCount);
		this.textureID = textureID;
	}

	public int getTexture(){
		return textureID;
	}
}
