package Motion.PathSystems;

import java.util.ArrayList;

import MathSystems.Vector2;
import MathSystems.Vector3;

public class Path {
	private ArrayList<Vector2> lines;
	private Vector3 endPoint;
	private Vector3 startPoint;

	public Path(Vector3 startPoint, Vector3 endPoint, ArrayList<Vector2> lines) {
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		this.lines = lines;
	}
	
	public Vector3 getStartpoint() {
		return startPoint;
	}
	
	public Vector3 getEndpoint() {
		return endPoint;
	}
	
	public ArrayList<Vector2> getPathLines(){
		return lines;
	}
}
