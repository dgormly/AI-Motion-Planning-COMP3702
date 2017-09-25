package agent;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
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
    public Set<Set<ASVConfig>> constructStateGraph(List<ASVConfig> vertices) {
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
        Set<Set<ASVConfig>> edges = new HashSet<>();
        adjacencyGraph = new int[size][size];

        for (ASVConfig c : vertices) {
            // Create Matrix
            List<ASVConfig> ASVsInRange = getASVsInRange(0.2, c, vertices);
            int pos = vertices.indexOf(c);
            for (ASVConfig d : ASVsInRange) {
                int pos2 = vertices.indexOf(d);
                adjacencyGraph[pos][pos2] = 1;
            }

            // Create graph
            for (int i = 0; i < size; i++) {
                for (int n = 0; n <= i; n++) {
                    if (adjacencyGraph[i][n] == 1) {
                        List<ASVConfig> segment = ASVConfig.createSegment(vertices.get(i), vertices.get(n));

                        if (!isValidSegment(segment)) {
                            continue;
                        }
                        Set<ASVConfig> list = new HashSet<>();
                        list.add(vertices.get(i));
                        list.add(vertices.get(n));
                        edges.add(list);
                    }
                }
            }
        }
        return edges;
    }


    /**
     *
     * @param cfg
     * @param vertices
     * @return
     */
    public ASVConfig getClosestASV(ASVConfig cfg, List<ASVConfig> vertices) {
        ASVConfig closestASV = null;
        double dist = 100;
        for (ASVConfig c : vertices) {
            if (c.equals(cfg)) {
                continue;
            }

            if (isValidSegment(ASVConfig.createSegment(cfg, c))) {
                double d = cfg.totalDistance(c);
                if (d < dist) {
                    closestASV = c;
                    dist = d;
                }
            }
        }
        return closestASV;
    }



    /**
     * Get vertices from map.
     *
     * @param distSize
     *      Distance to search.
     * @param vertices
     *      Vertices to scan through.
     * @return
     */
    public List<ASVConfig> getASVsInRange(double distSize, ASVConfig cfg, List<ASVConfig> vertices) {
        List<ASVConfig> returnList = new ArrayList<>();
        Point2D point = cfg.getPosition(0);

        for (ASVConfig c : vertices) {
            Point2D pp = c.getPosition(0);
            if (point.equals(pp)) {
                continue;
            }
            if (point.distance(pp) < distSize) {
                List<ASVConfig> segment = ASVConfig.createSegment(cfg, c);
                if (isValidSegment(segment)) {
                    returnList.add(c);
                }

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
    public List<Node> findPath(List<ASVConfig> vertices, ASVConfig start, ASVConfig goal) {
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


        ASVConfig closestStartPoint = getClosestASV(start, vertices);
        ASVConfig closestGoalPoint = getClosestASV(goal, vertices);

        /* Setup Search */
        // First node is start config, child is closest cfg.
        Node parent = new Node(null,  0, start);
        Node child = new Node(parent, closestStartPoint.totalDistance(closestGoalPoint), closestStartPoint);

        Set<ASVConfig> historySet = new HashSet<>();
        PriorityQueue<Node> container = new PriorityQueue<>();
        container.add(child);

        // Begin A* Star search.
        while (true) {
            if (container.size() == 0) {
                return null;
            }
            Node current = container.poll();
            if (historySet.contains(current.config)) {
                continue;
            }
            historySet.add(current.config);
            List<ASVConfig> children = getASVsInRange(0.2, current.config, vertices);

            for (ASVConfig c : children) {
                if (c.equals(closestGoalPoint)) {
                    /* Found goal config. */
                    List<Node> path = new ArrayList<>();
                    Node finalNode = new Node(current, c.totalDistance(current.config), goal);
                    path.add(finalNode);
                    while(current.parent != null) {
                        path.add(current);
                        current = current.parent;
                    }
                    path.add(current);
                    return path;
                }
                double cost = c.totalDistance(current.config) + c.totalDistance(closestGoalPoint);
                Node n = new Node(current, cost, c);
                container.add(n);
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
    public List<ASVConfig> sampleStateGraph(int numASVs,int sampleSize, Rectangle2D rect) {
        List<ASVConfig> sampleList = new ArrayList();
        for (int i = 0; i < sampleSize; i++) {
            Point2D point;

            do {
                double xPoint = Math.abs(Math.random() * rect.getWidth() + rect.getX());
                double yPoint = Math.abs(Math.random() * rect.getHeight() + rect.getY());
                point = new Point2D.Double(xPoint, yPoint);
            } while (!checkValidPoint(point, this.obstacles));

            List<ASVConfig> configs = ASVConfig.sampleConfigurations(point, numASVs, 1, obstacles);
            if (configs.size() == 0) {
                --i;
                continue;
            }
            sampleList.addAll(configs);
        }
        return sampleList;
    }


    /**
     * Checks if a segment is valid between two configurations.
     *
     * @param segment
     *      configuration path between one point and another.
     *
     * @return True if all configurations are valid in the segment.
     */
    public boolean isValidSegment(List<ASVConfig> segment) {
        for (ASVConfig asvConfig : segment) {
            if (!tester.isValidConfig(asvConfig, obstacles)) {
                return false;
            }
        }
        return true;
    }


    public List<ASVConfig> getSolution(List<Node> nodes) {
        List<ASVConfig> path = new ArrayList<>();
        List<ASVConfig> finalPath = new ArrayList<>();
        for (int i = nodes.size() - 1; i >= 0; i--) {
            path.add(nodes.get(i).config);
        }

        // Create segments
        for (int j = 0; j < path.size() - 1; j++) {
            ASVConfig c = path.get(j);
            ASVConfig cc = path.get(j + 1);
            List<ASVConfig> segment = ASVConfig.createSegment(c, cc);
            finalPath.addAll(segment);
        }
        return finalPath;
    }

    public void run() throws IOException {
        ASVConfig initialConfig = problemSpec.getInitialState();
        ASVConfig goalConfig = problemSpec.getGoalState();

        List<Node> path;
        List<ASVConfig> vertices = new ArrayList<>();

        do {
            vertices.addAll(sampleStateGraph(initialConfig.getASVCount(), 10, new Rectangle2D.Double(0, 0, 1, 1)));
            path = this.findPath(vertices, initialConfig, goalConfig);
        } while (path == null);

        problemSpec.setPath(getSolution(path));
        problemSpec.saveSolution("testing.txt");

        System.out.println("Solution Found!");
    }
}
