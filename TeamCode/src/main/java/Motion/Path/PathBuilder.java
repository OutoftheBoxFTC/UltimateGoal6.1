package Motion.Path;

import java.util.ArrayList;

import MathSystems.Angle;
import MathSystems.MathUtils;
import MathSystems.Vector2;
import MathSystems.Vector3;
import java.util.ArrayList;

import MathSystems.MathUtils;
import MathSystems.Vector2;

public class PathBuilder {
	private ArrayList<Vector3> lines;
	private Vector3 startPoint;
	
	public PathBuilder(Vector3 startPoint) {
		lines = new ArrayList<>();
		this.startPoint = startPoint;
		lines.add(startPoint);
	}

	public PathBuilder(double x, double y, Angle angle){
		lines = new ArrayList<>();
		this.startPoint = new Vector3(x, y, angle.radians());
		lines.add(startPoint);
	}

	public PathBuilder(Vector2 linPos, Angle angle){
		lines = new ArrayList<>();
		this.startPoint = linPos.toVector3(angle.radians());
		lines.add(startPoint);
	}
	
	public PathBuilder lineTo(Vector2 point, Angle angle) {
		lines.add(point.toVector3(angle.radians()));
		return this;
	}
	
	public PathBuilder lineTo(double x, double y, Angle r) {
		lines.add(new Vector3(x, y, r.radians()));
		return this;
	}
	
	public PathBuilder lineTo(Vector2 point) {
		lines.add(point.toVector3(lines.get(lines.size()-1).getC()));
		return this;
	}
	
	public PathBuilder lineTo(double x, double y) {
		lines.add(new Vector3(x, y, lines.get(lines.size()-1).getC()));
		return this;
	}
	
	public PathBuilder bezierSplineTo(Vector2 end, Vector2 control) {
		ArrayList<Vector2> vectors = (MathUtils.approxCurve(lines.get(lines.size()-1).getVector2(), end, control));
		double angle = lines.get(lines.size()-1).getC();
		for(Vector2 v : vectors) {
			lines.add(v.toVector3(angle));
		}
		return this;
	}
	
	public PathBuilder bezierSplineTo(Vector2 end, Vector2 control, Angle angle) {
		ArrayList<Vector2> vectors = (MathUtils.approxCurve(lines.get(lines.size()-1).getVector2(), end, control));
		double maxLen = MathUtils.arcLength(vectors);
		double endRot = angle.radians();
		double startRot = lines.get(lines.size()-1).getC();
		for(int i = 0; i < vectors.size(); i ++) {
			double len = MathUtils.arcLength(vectors.subList(i, vectors.size()));
			lines.add(vectors.get(i).toVector3(((startRot - endRot) * (len/maxLen)) + endRot));
		}
		return this;
	}
	
	public Path complete() {
		return new Path(startPoint, lines.get(lines.size()-1), lines);
	}
}
