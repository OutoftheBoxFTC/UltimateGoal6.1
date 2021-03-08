package Motion.PathSystems;

import java.util.ArrayList;

import MathSystems.Angle;
import MathSystems.MathUtils;
import MathSystems.Vector2;
import MathSystems.Vector3;
import java.util.ArrayList;

import MathSystems.MathUtils;
import MathSystems.Vector2;

public class PathBuilder {
	private ArrayList<Vector2> lines;
	private Vector3 startPoint;
	private Angle endAngle;
	
	public PathBuilder(Vector3 startPoint) {
		lines = new ArrayList<>();
		this.startPoint = startPoint;
		lines.add(startPoint.getVector2());
		endAngle = Angle.radians(startPoint.getC());
	}

	public PathBuilder(double x, double y, Angle angle){
		lines = new ArrayList<>();
		this.startPoint = new Vector3(x, y, angle.radians());
		lines.add(startPoint.getVector2());
		endAngle = angle;
	}
	
	public PathBuilder lineTo(Vector2 point) {
		lines.add(point);
		return this;
	}
	
	public PathBuilder lineTo(double x, double y) {
		lines.add(new Vector2(x, y));
		return this;
	}
	
	public PathBuilder bezierSplineTo(Vector2 end, Vector2 control) {
		lines.addAll(MathUtils.approxCurve(lines.get(lines.size()-1), end, control));
		return this;
	}
	
	public PathBuilder setAngle(Angle angle) {
		endAngle = angle;
		return this;
	}
	
	public Path complete() {
		return new Path(startPoint, lines.get(lines.size()-1).toVector3(endAngle.radians()), lines);
	}
}
