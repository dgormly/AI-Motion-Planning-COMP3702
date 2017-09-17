package agent;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;
import java.util.List;

public class SearchAgent {

    private ProblemSpec problemSpec;
    private List<Obstacle> obstacleList;
    public Map<Point2D, List<Point2D>> graph;
    public List<Point2D> sampleList;
    public int[][] adjacenyGraph;


    public SearchAgent(ProblemSpec problem) {
        problemSpec = problem;
        obstacleList = problem.getObstacles();
        obstacleList.forEach((o) -> {
            Rectangle2D rect = o.getRect();
        });
        graph = new HashMap<>();
        sampleList = new ArrayList<>();

        //List<Point2D> initState = problemSpec.getInitialState().getASVPositions();
        //List<Point2D> finalState = problemSpec.getGoalState().getASVPositions();
        //sampleList.add(initState.get(0));
        //sampleList.add(finalState.get(0));

        //graph.put(initState.get(0), initState);
        //graph.put(finalState.get(0), finalState);
    }

    public List<Point2D> samplePoints(int sampleSize, double x, double y, double x2, double y2) {
        sampleList = new ArrayList();
        for (int i = 0; i < sampleSize; i++) {
            Point2D point;
            do {
                double xPoint = Math.abs(Math.random() * x2 + x);
                double yPoint = Math.abs(Math.random() * y2 + y);
                point = new Point2D.Double(xPoint, yPoint);
            } while (checkValidPoint(point));
            sampleList.add(point);
        }
        return sampleList;
    }

    public List<List<Point2D>> createGraph(List<Point2D> vertices) {
        int size = vertices.size();
        Set<Set<Point2D>> edges = new HashSet<>();
        adjacenyGraph = new int[size][size];
        for (Point2D p : vertices) {
            List<Point2D> points = getPointsInRange(0.1, p, vertices);
            int pos = vertices.indexOf(p);
            for (Point2D p2 : points) {
                int pos2 = vertices.indexOf(p2);
                adjacenyGraph[pos][pos2] = 1;
            }
            // Great graph
            for (int i = 0; i < size; i++) {
                for (int n = 0; n <= i; n++) {
                    if (adjacenyGraph[i][n] == 1) {
                        Line2D line = new Line2D.Double(vertices.get(i), vertices.get(n));
                        boolean clash = false;
                        for (Obstacle o : obstacleList) {
                            if (line.intersects(o.getRect())) {
                                clash = true;
                            }
                        }
                        if (clash) {
                            continue;
                        }
                        Set<Point2D> list = new HashSet<>();
                        list.add(vertices.get(i));
                        list.add(vertices.get(n));
                        edges.add(list);
                    }
                }
            }
        }
        List<List<Point2D>> edgeList = new ArrayList<>();
        for (Set<Point2D> e : edges) {
            List<Point2D> l = new ArrayList<>();
            l.addAll(e);
            edgeList.add(l);
        }
        return edgeList;
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
                return true;
            }
        }
        return false;
    }


    public Point2D getClosestPoint(Point2D point, List<Point2D> vertices) {
        Point2D closestPoint = new Point2D.Double();
        double dist = 100;
        for (Point2D p : vertices) {
            if (p.equals(point)) {
                continue;
            }
            double d = point.distance(p);
            if (d < dist) {
                closestPoint = p;
                dist = d;
            }
        }
        return closestPoint;
    }

    public List<Point2D> getPointsInRange(double distSize, Point2D point, List<Point2D> vertices) {
        List<Point2D> returnList = new ArrayList<>();
        for (Point2D p : vertices) {
            if (point.equals(p)) {
                continue;
            }
            if (point.distance(p) < distSize) {
                returnList.add(p);
            }

        }
        return returnList;
    }

    public List<Point2D> findPath(List<Point2D> vertices, Point2D start, Point2D goal) {
        Point2D closestStartPoint = getClosestPoint(start, vertices);
        Point2D closestGoalPoint = getClosestPoint(goal, vertices);

        // Start Search
        int pos = vertices.indexOf(closestStartPoint);
        Node parent = new Node(null, closestStartPoint);
        Set<Point2D> historySet = new HashSet<>();
        LinkedList<Node> queue = new LinkedList<>();
        queue.add(parent);
        while (true) {
            Node current = queue.poll();
            historySet.add(current.point);
            List<Point2D> children = getPointsInRange(0.1, current.point, vertices);
            children.remove(current.point);
            for (Point2D p : children) {
                if (p.equals(closestGoalPoint)) {
                    List<Point2D> path = new ArrayList<>();
                    Node finalNode = new Node(current, p);
                    path.add(finalNode.point);
                    while(current.parent != null) {
                        path.add(current.point);
                        current = current.parent;
                    }
                    path.add(current.point);
                    return path;
                }
                Node n = new Node(current, p);
                queue.push(n);
            }
        }
    }
}
