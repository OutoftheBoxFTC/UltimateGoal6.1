package Motion.Path;

import java.util.ArrayList;

import MathSystems.Vector2;
import MathSystems.Vector3;

public class Path {
	private ArrayList<Vector3> lines;
	private ArrayList<Vector2> pathLines;
	private Vector3 endPoint;
	private Vector3 startPoint;

	public Path(Vector3 startPoint, Vector3 endPoint, ArrayList<Vector3> lines) {
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		this.lines = lines;
		pathLines = new ArrayList<Vector2>();
		for(Vector3 v : lines) {
			pathLines.add(v.getVector2());
		}
	}
	
	public Vector3 getStartpoint() {
		return startPoint;
	}
	
	public Vector3 getEndpoint() {
		return endPoint;
	}
	
	public ArrayList<Vector2> getPathLines(){
		return pathLines;
	}
	
	public ArrayList<Vector3> getLines(){
		return lines;
	}
}
