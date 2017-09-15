package agent;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class SearchAgent {

    private ProblemSpec problemSpec;
    private ASVConfig config;
    private List<Obstacle> obstacleList;

    public SearchAgent(ProblemSpec problem) {
        problemSpec = problem;
        config = problem.getInitialState();
        obstacleList = problem.getObstacles();
    }

    public List<Point2D> distributePoints(double num, double x, double y, double x2, double y2) {
        List<Point2D> list = new ArrayList<>();
        for (int i = 0; i < num; i++) {
            double xPoint = Math.abs(Math.random() * x2 + x);
            double yPoint = Math.abs(Math.random() * y2 + y);
            Point2D point = new Point2D.Double(xPoint, yPoint);
            if (checkValidPoint(point)) {
                list.add(point);
            }
        }
        return list;
    }

    public boolean checkValidPoint(Point2D point) {
        double x = point.getX();
        double y = point.getY();
        for (Obstacle o: obstacleList) {
            double ox = o.getRect().getX();
            double oy = o.getRect().getY();
            double width = o.getRect().getWidth();
            double height = o.getRect().getHeight();

            if ((x > ox && x < ox + width) &&
                    (y > oy && y < oy + height)) {
                return false;
            }
        }
        return true;
    }

}
