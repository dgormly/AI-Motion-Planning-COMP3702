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

    public List<Point2D> sampleList;
    public Tester tester = new Tester();
    private Sampler sampler;


    public SearchAgent(ProblemSpec problem) {
        problemSpec = problem;
        tester.ps = problemSpec;
        obstacles = problemSpec.getObstacles();
        sampleList = new ArrayList<>();
        sampler = new Sampler(obstacles, new Rectangle2D.Double(0,0,1,1));
    }


//    /**
//     * Needs completely rewritting.
//     *
//     * Creates a graph from the given List.
//     *
//     * @param vertices
//     * @return
//     */
//    public Set<Set<ASVConfig>> constructStateGraph(List<ASVConfig> vertices) {
//        /*
//        Loop
//            Sample a configuration q uniformly at random from the state space. - Insert sampling stategy.
//            if q is not in collision.
//                Add q as a vertice to the state graph.
//                For all q' within D distance from q in state graph.
//                    If the line segment  (in C space) between q and q’ is not in-collision,
//                     add an edge qq’ to the state graph.
//         */
//
//        int size = vertices.size();
//        Set<Set<ASVConfig>> edges = new HashSet<>();
//        adjacencyGraph = new int[size][size];
//
//        for (ASVConfig c : vertices) {
//            // Create Matrix
//            List<ASVConfig> ASVsInRange = getASVsInRange(0.2, c, vertices);
//            int pos = vertices.indexOf(c);
//            for (ASVConfig d : ASVsInRange) {
//                int pos2 = vertices.indexOf(d);
//                adjacencyGraph[pos][pos2] = 1;
//            }
//
//            // Create graph
//            for (int i = 0; i < size; i++) {
//                for (int n = 0; n <= i; n++) {
//                    if (adjacencyGraph[i][n] == 1) {
//                        List<ASVConfig> segment = ASVConfig.createSegment(vertices.get(i), vertices.get(n));
//
//                        if (!isValidSegment(segment)) {
//                            continue;
//                        }
//                        Set<ASVConfig> list = new HashSet<>();
//                        list.add(vertices.get(i));
//                        list.add(vertices.get(n));
//                        edges.add(list);
//                    }
//                }
//            }
//        }
//        return edges;
//    }


    /**
     * Returns the closes ASV with a valid segment.
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

        // Sample around starting areas.
        ASVConfig closestStartPoint = getClosestASV(start, vertices);
        ASVConfig closestGoalPoint = getClosestASV(goal, vertices);

        if (closestGoalPoint == null || closestStartPoint == null) {
            return null;
        }

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
            List<ASVConfig> children = getASVsInRange(0.3, current.config, vertices);

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


    /**
     * Returns the soultion path.
     *
     * @param nodes
     *      List of nodes to be converted into ASVConfigs.
     *
     * @return ASV Configuration path.
     */
    public List<ASVConfig> getSolution(List<Node> nodes) {

        List<ASVConfig> path = new ArrayList<>();
        List<ASVConfig> finalPath = new ArrayList<>();

        for (int i = nodes.size() - 1; i >= 0; i--) {
            path.add(nodes.get(i).config);
        }

        // Create segments
        List<ASVConfig> newPath = new ArrayList<>();
        newPath.add(path.get(0));
        for (int j = 1; j < path.size(); j++) {
            ASVConfig c = path.get(j);
            ASVConfig cc = newPath.get(newPath.size() - 1);
            List<ASVConfig> segment = ASVConfig.createSegment(c, cc);

            if (!isValidSegment(segment)) {
                newPath.add(path.get(j - 1));
            }
        }
        newPath.add(path.get(path.size() - 1));

        for (int j = 0; j < newPath.size() - 1; j++) {
            ASVConfig c = newPath.get(j);
            ASVConfig cc = newPath.get(j + 1);

            List<ASVConfig> segment = ASVConfig.createSegment(c, cc);
            finalPath.addAll(segment);
        }
        return finalPath;
    }


    /**
     * Controls the AI search.
     *
     * @throws IOException
     */
    public void run() throws IOException {

        ASVConfig initialConfig = problemSpec.getInitialState();
        ASVConfig goalConfig = problemSpec.getGoalState();

        List<Node> path;
        List<ASVConfig> vertices = new ArrayList<>();

        do {
            vertices.addAll(sampler.sampleUniformly(initialConfig.getASVCount(), 500));
            path = this.findPath(vertices, initialConfig, goalConfig);
        } while (path == null);

        problemSpec.setPath(getSolution(path));
        problemSpec.saveSolution("testing.txt");

        System.out.println("Solution Found!");
    }
}
