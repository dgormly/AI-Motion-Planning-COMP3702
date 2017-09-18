package agent;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

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
        obstacleList = problemSpec.getObstacles();
        graph = new HashMap<>();
        sampleList = new ArrayList<>();


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
            List<Point2D> points = getPointsInRange(0.05, p, vertices);
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
        Node parent = new Node(null, closestStartPoint, closestStartPoint.distance(closestGoalPoint));
        Set<Point2D> historySet = new HashSet<>();
        PriorityQueue<Node> container = new PriorityQueue<>();
        container.add(parent);
        while (true) {
            Node current = container.poll();
            if (historySet.contains(current.point)) {
                continue;
            }
            historySet.add(current.point);
            List<Point2D> children = getPointsInRange(0.05, current.point, vertices);
            for (Point2D p : children) {
                if (p.equals(closestGoalPoint)) {
                    List<Point2D> path = new ArrayList<>();
                    Node finalNode = new Node(current, p, p.distance(current.point));
                    path.add(finalNode.point);
                    while(current.parent != null) {
                        path.add(current.point);
                        current = current.parent;
                    }
                    path.add(current.point);
                    return path;
                }
                double cost = p.distance(current.point) + p.distance(closestGoalPoint);
                Node n = new Node(current, p, cost);
                container.add(n);
            }
        }
    }

    public List<ASVConfig> findInvalidConfigs(ASVConfig config, List<Point2D> path, List<Obstacle> obstacles) {
        Tester tester = new Tester();
        ASVConfig pathConfig;
        List<ASVConfig> invalidStates = new ArrayList<>();
        for (Point2D p : path) {
            pathConfig = moveASV(config, p);
             if (tester.hasCollision(pathConfig, obstacles)) {
                 invalidStates.add(new ASVConfig(pathConfig));
             }
        }
        return invalidStates;
    }

    public ASVConfig moveASV(ASVConfig config, Point2D newPos) {
        ASVConfig newConfig = new ASVConfig(config);
        Point2D anchorPoint = config.getPosition(0);
        // Configure asvs relative to parent.
        for (int i = 1; i < newConfig.getASVCount(); i++) {
            Point2D p = newConfig.getPosition(i);
            double differenceX = p.getX() - anchorPoint.getX();
            double differenceY = p.getY() - anchorPoint.getY();
            p.setLocation(newPos.getX() + differenceX, newPos.getY() + differenceY);
        }
        newConfig.getPosition(0).setLocation(newPos);
        return newConfig;
    }   
}
