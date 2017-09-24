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
    private List<Obstacle> obstacles;
    public List<Obstacle> expandedList;
    public Map<Point2D, List<Point2D>> graph;
    public List<Point2D> sampleList;
    public int[][] adjacencyGraph;
    public Tester tester = new Tester();
    public List<ASVConfig> asvPath = new ArrayList<>();


    public SearchAgent(ProblemSpec problem) {
        problemSpec = problem;
        tester.ps = problemSpec;
        obstacles = problemSpec.getObstacles();
        expandedList = new ArrayList<>();
        for (Obstacle obstacle : obstacles) {
            Obstacle o = new Obstacle(tester.grow(obstacle.getRect(), 0.01));
            expandedList.add(o);
        }
        graph = new HashMap<>();
        sampleList = new ArrayList<>();
    }


    /**
     * Needs completely rewritting.
     *
     * Creates a graph from the given List.
     *
     * @param vertices
     * @return
     */
    public List<List<Point2D>> constructStateGraph(List<Point2D> vertices) {
        /*
        Loop
            Sample a configuration q uniformly at random from the state space. - Insert sampling stategy.
            if q is not in collision.
                Add q as a vertice to the state graph.
                For all q' within D distance from q in state graph.
                    If the line segment  (in C space) between q and q’ is not in-collision,
                     add an edge qq’ to the state graph.
         */

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
            // Create graph
            for (int i = 0; i < size; i++) {
                for (int n = 0; n <= i; n++) {
                    if (adjacencyGraph[i][n] == 1) {
                        Line2D line = new Line2D.Double(vertices.get(i), vertices.get(n));
                        boolean clash = false;
                        for (Obstacle o : obstacles) {
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
            for (Obstacle obstacle : obstacles) {
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



    /**
     * Get vertices from map.
     *
     * @param distSize
     *      Distance to search.
     * @param point
     *      Vertice to scan around.
     * @param vertices
     *      Vertices to scan through.
     * @return
     */
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

    /**
     * Finds a path from a list of vertice using A* search
     * Heuristic distance to the final config.
     *
     * @param vertices
     *      to search through.
     * @param start
     *      the starting configuration.
     * @param goal
     *      the end configuration.
     * @return List of Nodes containing the path.
     */
    public List<Node> findPath(List<Point2D> vertices, ASVConfig start, ASVConfig goal) {
        /*
        While runtime < timeLimit AND path is not found repeat
        Sample a configuration q with a suitable sampling strategy. if q is collision-free then
        Add q to the graph G.
        Connect q to existing vertices in G using valid edges. until n new vertices have been added to G.
                Search G for a path.
        */

        /*
        Given an initial & a goal configurations,
        Find the vertex qi nearest to the initial configuration,
            where the straight line segment between initial configuration & qi is collision free.
        Find the vertex qg nearest to the goal configuration,
            where the straight line segment between goal configuration & qg is collision free.
        Find a path from qi to qg using the search algorithms we have discussed (blind/informed search).
         */


        Point2D closestStartPoint = getClosestPoint(start.getPosition(0), vertices);
        Point2D closestGoalPoint = getClosestPoint(goal.getPosition(0), vertices);

        // Setup Search
        Node parent = new Node(null, start.getPosition(0), 0, start);
        Node child = new Node(parent, closestStartPoint, closestStartPoint.distance(closestGoalPoint), start);
        Set<Point2D> historySet = new HashSet<>();
        PriorityQueue<Node> container = new PriorityQueue<>();
        container.add(parent);
        container.add(child);

        // Begin A* Star search.
        while (true) {
            Node current = container.poll();
            if (historySet.contains(current.point)) {
                continue;
            }
            historySet.add(current.point);
            List<Point2D> children = getPointsInRange(0.05, current.point, vertices);
            for (Point2D p : children) {
                if (p.equals(closestGoalPoint)) {
                    /* Found goal config. */
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
//                ASVConfig newConfig = new ASVConfig(current.config);
//                newConfig.move(p);
//                Node n = new Node(current, p, cost, newConfig);
//                container.add(n);
            }
        }
    }


    /**
     * Checks if a given point collides with any obstacles.
     *
     * @param point
     *      checks if a point is in a valid position.
     * @return True if point is in an obstacle.
     */
    private static boolean checkValidPoint(Point2D point, List<Obstacle> obstacles) {
        double x = point.getX();
        double y = point.getY();
        for (Obstacle o: obstacles) {
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


    /**
     *
     * @param sampleSize
     * @param rect
     * @return
     */
    public List<Point2D> sampleStateGraph(int sampleSize, Rectangle2D rect) {
        List<Point2D> sampleList = new ArrayList();
        Tester tester = new Tester();
        for (int i = 0; i < sampleSize; i++) {
            Point2D point;
            do {
                double xPoint = Math.abs(Math.random() * rect.getWidth() + rect.getX());
                double yPoint = Math.abs(Math.random() * rect.getHeight() + rect.getY());
                point = new Point2D.Double(xPoint, yPoint);
            } while (!checkValidPoint(point, this.obstacles));
            sampleList.add(point);
        }
        return sampleList;
    }


    /**
     * Checks if a segment is valid between two configurations.
     *
     * @param cfg1
     * @param cfg2
     * @return True if all configurations are valid in the segment.
     */
    public boolean isValidSegment(ASVConfig cfg1, ASVConfig cfg2) {
        List<ASVConfig> segemnt = ASVConfig.transform(cfg1, cfg2);
        for (ASVConfig asvConfig : segemnt) {
            if (!tester.isValidConfig(asvConfig, obstacles)) {
                return false;
            }
        }
        return true;
    }

    public void run() {
//        ASVConfig initialConfig = problemSpec.getInitialState();
//        ASVConfig goalConfig = problemSpec.getGoalState();
//        goalConfig.rotate(1, 90);
//        List<ASVConfig> path = ASVConfig.transform(initialConfig, goalConfig);
//        problemSpec.setPath(path);
    }
}
