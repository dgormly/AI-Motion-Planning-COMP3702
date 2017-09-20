package agent;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.List;

public class SearchAgent {

    private ProblemSpec problemSpec;
    private List<Obstacle> obstacleList;
    public Map<Point2D, List<Point2D>> graph;
    public List<Point2D> sampleList;
    public int[][] adjacencyGraph;
    public Tester tester = new Tester();
    public List<ASVConfig> asvPath = new ArrayList<>();


    public SearchAgent(ProblemSpec problem) {
        problemSpec = problem;
        obstacleList = problemSpec.getObstacles();
        graph = new HashMap<>();
        sampleList = new ArrayList<>();
        tester.ps = problemSpec;
    }

    public List<Point2D> samplePoints(int sampleSize, double x, double y, double x2, double y2) {
        sampleList = new ArrayList();
        for (int i = 0; i < sampleSize; i++) {
            Point2D point;
            do {
                double xPoint = Math.abs(Math.random() * x2 + x);
                double yPoint = Math.abs(Math.random() * y2 + y);
                point = new Point2D.Double(xPoint, yPoint);
            } while (!checkValidPoint(point));
            sampleList.add(point);
        }
        return sampleList;
    }

    public List<List<Point2D>> createGraph(List<Point2D> vertices) {
        int size = vertices.size();
        Set<Set<Point2D>> edges = new HashSet<>();
        adjacencyGraph = new int[size][size];
        for (Point2D p : vertices) {
            List<Point2D> points = getPointsInRange(0.05, p, vertices);
            int pos = vertices.indexOf(p);
            for (Point2D p2 : points) {
                int pos2 = vertices.indexOf(p2);
                adjacencyGraph[pos][pos2] = 1;
            }
            // Great graph
            for (int i = 0; i < size; i++) {
                for (int n = 0; n <= i; n++) {
                    if (adjacencyGraph[i][n] == 1) {
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
                return false;
            }
        }
        return true;
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

    public List<Node> findPath(List<Point2D> vertices, ASVConfig start, ASVConfig goal) {
        Point2D closestStartPoint = getClosestPoint(start.getPosition(0), vertices);
        Point2D closestGoalPoint = getClosestPoint(goal.getPosition(0), vertices);

        // Start Search
        Node parent = new Node(null, start.getPosition(0), 0, start);
        Node child = new Node(parent, closestStartPoint, closestStartPoint.distance(closestGoalPoint), start);
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
                    List<Node> path = new ArrayList<>();
                    Node finalNode = new Node(current, p, p.distance(current.point), goal);
                    path.add(finalNode);
                    while(current.parent != null) {
                        path.add(current);
                        current = current.parent;
                    }
                    path.add(current);
                    return path;
                }
                double cost = p.distance(current.point) + p.distance(closestGoalPoint);
                ASVConfig newConfig = new ASVConfig(current.config);
                newConfig = moveASV(newConfig, p);
                Node n = new Node(current, p, cost, newConfig);
                container.add(n);
            }
        }
    }

    public List<ASVConfig> generateConfigs(ASVConfig originalConfig) {
        List<ASVConfig> validConfigs = new ArrayList<>();
        while (validConfigs.size() < 50) {
            ASVConfig newConfig = new ASVConfig(originalConfig);
            double range = 180 + (newConfig.getASVCount() - 3) * 180;
            for (int i = 1; i < originalConfig.getASVCount() - 1; i++) {
                double initialAngleDegrees = Math.random() * range * 2 - range;
                rotateASV(newConfig, i, (int) initialAngleDegrees);
                range -= initialAngleDegrees;
            }
            rotateASV(newConfig, newConfig.getASVCount() - 1, (int) range);
            if (tester.isValidConfig(newConfig, obstacleList)) {
                validConfigs.add(newConfig);
            }
        }
        return validConfigs;
    }

    public ASVConfig getBestConfig(ASVConfig invalidConfig, List<ASVConfig> list) {
        double dist = 2;
        ASVConfig best = new ASVConfig(list.get(0));
        for (ASVConfig asvConfig : list) {
            double tempDist = 0;
            for (int i = 0; i < asvConfig.getASVPositions().size(); i++) {
                dist += asvConfig.getPosition(i).distance(invalidConfig.getPosition(i));
            }
            if (tempDist < dist ) {
                best = asvConfig;
            }
        }
        return best;
    }

    private ASVConfig moveASV(ASVConfig config, Point2D newPos) {
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

    private void rotatePoint(Point2D anchorPoint, Point2D point, int degree) {
        double h = anchorPoint.distance(point);
        double currentRad = Math.toRadians(degree);

        // New Pos
        double newX = h * Math.cos(currentRad) + anchorPoint.getX();
        double newY = h * Math.sin(currentRad) + anchorPoint.getY();
        point.setLocation(newX, newY);
    }

    private void rotateASV(ASVConfig config, int pointNumber, int degrees) {
        ASVConfig oc = new ASVConfig(config);
        for (int i = pointNumber; i < config.getASVCount(); i++) {
            Point2D p = config.getPosition(i);
            rotatePoint(oc.getPosition(i - 1), p, degrees);
        }
    }

    public List<ASVConfig> transform(ASVConfig initialCfg, ASVConfig goalConfig) {
        List<ASVConfig> finalSolution = new ArrayList<>();
        finalSolution.add(initialCfg);
        while (true) {
            ASVConfig cfg = new ASVConfig(finalSolution.get(finalSolution.size() - 1));
            for (int i = 0; i < initialCfg.getASVCount(); i++) {
                double yDist = goalConfig.getPosition(i).getY() - cfg.getPosition(i).getY();
                double xDist = goalConfig.getPosition(i).getX() - cfg.getPosition(i).getX();
                double distance = goalConfig.getPosition(i).distance(cfg.getPosition(i));
                double angle = Math.asin(yDist / distance);
                double stepSize = distance > 0.001 ? 0.001 : distance;
                double xRate = stepSize * Math.cos(angle);
                double yRate = stepSize * Math.sin(angle);
                double x = cfg.getPosition(i).getX() + xRate;
                double y = cfg.getPosition(i).getY() + yRate;

                if (xDist > 0.001 || yDist > 0.001) {
                    cfg.getPosition(i).setLocation(x, y);
                } else {
                    cfg.getPosition(i).setLocation(x, y);
                    finalSolution.add(cfg);
                    return finalSolution;
                }
            }
            finalSolution.add(cfg);
        }
    }

    public void run() {
        List<Point2D> sample = samplePoints(2000,0, 0, 1, 1);
        ASVConfig initialConfig = problemSpec.getInitialState();
        ASVConfig goalConfig = problemSpec.getGoalState();

        List<Node> path = findPath(sample, initialConfig, goalConfig);
        List<ASVConfig> asvPath = new ArrayList<>();
        for (Node node : path) {
            asvPath.add(node.config);
        }
        for (ASVConfig asvConfig : asvPath) {
            if (!tester.isValidConfig(asvConfig, obstacleList)) {
                List<ASVConfig> possibleConfigs = generateConfigs(asvConfig);
                ASVConfig best = getBestConfig(asvConfig, possibleConfigs);
                for (int c = asvPath.indexOf(asvConfig); c < asvPath.size(); c++) {
                    asvPath.set(c, best);
                }
            }
        }

        List<ASVConfig> finalPath = new ArrayList<>();
        for (int i = 0; i < asvPath.size() - 1; i++) {
            ASVConfig initial = asvPath.get(i);
            ASVConfig goal = asvPath.get(i + 1);
            finalPath.addAll(transform(initial, goal));
        }
        problemSpec.setPath(finalPath);
    }
}
