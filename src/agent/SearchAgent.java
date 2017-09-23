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
    public List<Obstacle> expandedList;
    public Map<Point2D, List<Point2D>> graph;
    public List<Point2D> sampleList;
    public int[][] adjacencyGraph;
    public Tester tester = new Tester();
    public List<ASVConfig> asvPath = new ArrayList<>();


    public SearchAgent(ProblemSpec problem) {
        problemSpec = problem;
        tester.ps = problemSpec;
        obstacleList = problemSpec.getObstacles();
        expandedList = new ArrayList<>();
        for (Obstacle obstacle : obstacleList) {
            Obstacle o = new Obstacle(tester.grow(obstacle.getRect(), 0.01));
            expandedList.add(o);
        }
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
        for (Obstacle o: expandedList) {
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
            Line2D line = new Line2D.Double();
            line.setLine(point, p);
            boolean intersect = false;
            for (Obstacle obstacle : obstacleList) {
                if (line.intersects(obstacle.getRect())) {
                    intersect = true;
                }
            }
            if (intersect) {
                continue;
            }
            if (d < dist) {
                closestPoint = p;
                dist = d;
            }
        }
        return closestPoint;
    }

    public List<Point2D> joinPoints(Point2D point1, Point2D point2) {
        List<Point2D> points = new ArrayList<>();
        points.add(point1);
        double distance = point1.distance(point2) / 0.001;
        double xRate = (point2.getX() - point1.getX()) / distance;
        double yRate = (point2.getY() - point1.getY()) / distance;
        for (int i = 0; i < distance; i++) {
            Point2D prevPoint = points.get(i);
            Point2D point = new Point2D.Double(xRate + prevPoint.getX(), yRate + prevPoint.getY());
            points.add(point);
        }
        return points;
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
        container.add(child);
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
        List<Double> validRangle = new ArrayList<>();
        /* Fine tune this to get the right distances in test. */
        validRangle.add(0.05);
        validRangle.add(0.00);
        validRangle.add(-0.05);

        for (int i = 1; i < originalConfig.getASVPositions().size(); i++) {
            for (Double aDouble : validRangle) {
                ASVConfig c = new ASVConfig(originalConfig);
                rotateASV(c, i, aDouble);
                if (tester.isValidConfig(c, obstacleList)) {
                    validConfigs.add(c);
                }
            }
        }
        return validConfigs;
    }

    public ASVConfig getBestConfig(ASVConfig invalidConfig, List<ASVConfig> list) {
        double dist = 2;
        ASVConfig best = new ASVConfig(list.get(0));
        for (ASVConfig asvConfig : list) {
            double tempDist = invalidConfig.totalDistance(asvConfig);
            if (tempDist < dist) {
                dist = tempDist;
            }
        }
        return best;
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

    private void rotatePoint(Point2D anchorPoint, Point2D point, double degree) {
        double h = anchorPoint.distance(point);
        double rad = Math.toRadians(degree);
        double changeX = point.getX() - anchorPoint.getX();
        double changeY = point.getY() - anchorPoint.getY();
        double currentRad = Math.atan2(changeY, changeX);

        // New Pos
        double newX = h * Math.cos(rad + currentRad) + anchorPoint.getX();
        double newY = h * Math.sin(currentRad + rad) + anchorPoint.getY();
        point.setLocation(newX, newY);
    }

    private void rotateASV(ASVConfig config, int pointNumber, double degrees) {
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
                double angle = Math.atan2(yDist , xDist);
                double distance = goalConfig.getPosition(i).distance(cfg.getPosition(i));
                double stepDist = distance > 0.001 ? 0.001 : Math.abs(distance);

                double xRate = stepDist * Math.cos(angle);
                double yRate = stepDist * Math.sin(angle);

                double x = cfg.getPosition(i).getX() + xRate;
                double y = cfg.getPosition(i).getY() + yRate;
                cfg.getPosition(i).setLocation(x, y);
                if (cfg.totalDistance(goalConfig) < 0.001) {
                    return finalSolution;
                }
            }
            finalSolution.add(cfg);
        }
    }

    public void run() {
        List<Point2D> sample = samplePoints(5000, 0, 0, 1, 1);
        ASVConfig initialConfig = problemSpec.getInitialState();
        ASVConfig goalConfig = problemSpec.getGoalState();

        List<Node> path = findPath(sample, initialConfig, goalConfig);
        List<ASVConfig> asvPath = new ArrayList<>();
        for (Node node : path) {
            asvPath.add(node.config);
        }

        boolean invalid = true;
        while (invalid) {
            for (ASVConfig asvConfig : asvPath) {
                if (!tester.isValidConfig(asvConfig, obstacleList)) {
                    List<ASVConfig> possibleConfigs = generateConfigs(asvConfig);
                    ASVConfig best = getBestConfig(asvConfig, possibleConfigs);
                    for (int c = asvPath.indexOf(asvConfig); c < asvPath.size(); c++) {
                        ASVConfig temp = asvPath.get(c);
                        temp = moveASV(best, temp.getPosition(0));
                        asvPath.set(c, temp);
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

    public List<Node> searchConfigs(List<Point2D> path, ASVConfig cfg) {
        // Start Search
        System.out.println(path.size());
        Node parent = new Node(null, path.get(0), 0, cfg);
        Set<ASVConfig> historySet = new HashSet<>();
        Stack<Node> container = new Stack<>();
        container.add(parent);

        while (true) {
            Node current = container.pop();
            if (historySet.contains(current.config)) {
                continue;
            }
            historySet.add(current.config);
            int pointIndex = path.indexOf(current.point) + 1;
            if (pointIndex == path.size()) {
                List<Node> finalPath = new ArrayList<>();
                while (current.parent != null) {
                    finalPath.add(current);
                    current = current.parent;
                }
                finalPath.add(current);
                System.out.println("SOLUTION FOUND");
                return finalPath;
            }
                    /* Create nodes of all possible configs. */
            List<ASVConfig> configList = generateConfigs(current.config);
            for (ASVConfig asvConfig : configList) {
                ASVConfig c = moveASV(asvConfig, path.get(pointIndex));
                Node n = new Node(current, path.get(path.indexOf(current.point)), c.totalDistance(current.config), c);
                container.add(n);
            }
        }
    }
}
