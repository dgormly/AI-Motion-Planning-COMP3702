package agent;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.*;
import java.util.List;

public class SearchAgent {

    private final double SOLUTION_STEP_SIZE = 0.001;
    private final double SEARCH_RANGE = 0.2;
    private final int SAMPLE_SIZE = 200;
    private final int CLASH_SAMPLES = 5;

    private ProblemSpec problemSpec;
    private List<Obstacle> obstacles;
    public List<Edge> forbiddenEdges = new ArrayList<>();
    private List<Edge> significantEdge = new ArrayList<>();

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

            if (isValidSegment(ASVConfig.createSegment(cfg, c, SOLUTION_STEP_SIZE))) {
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

        for (ASVConfig c : vertices) {
            if (cfg.equals(c)) {
                continue;
            }
            if (cfg.totalDistance(c) < distSize) {
                returnList.add(c);

            }
        }
        return returnList;
    }

    /**
     * Finds a path from a list of vertice using A* search
     * Heuristic distance to the final config.
     * Does not check for clashes.
     *
     * @param vertices
     *      to search through.
     * @param start
     *      the starting configuration.
     * @param goal
     *      the end configuration.
     * @return List of Nodes containing the path.
     */
    public List<Node> aStarSearch(List<ASVConfig> vertices, ASVConfig start, ASVConfig goal) {
        ASVConfig closestGoalPoint = goal;

        /* Setup Search */
        // First node is start config, child is closest cfg.
        Node parent = new Node(null,  0, start);

        Set<ASVConfig> historySet = new HashSet<>();
        PriorityQueue<Node> container = new PriorityQueue<>();
        container.add(parent);

        Node current;
        List<Node> path = new ArrayList<>();
        // Begin A* Star search.
        while (true) {
            if (container.size() == 0) {
                return null;
            }
            current = container.poll();
            if (historySet.contains(current.config)) {
                continue;
            }

            historySet.add(current.config);
            List<ASVConfig> children = getASVsInRange(SEARCH_RANGE, current.config, vertices);

            for (ASVConfig c : children) {
                Edge e = new Edge(current.config, c);
                if (forbiddenEdges.contains(e)) {
                    continue;
                }

                if (c.equals(closestGoalPoint)) {
                    /* Found goal config. */
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


    public void expansionPhase(List<ASVConfig> vertices, ASVConfig initialCfg, ASVConfig goalCfg) {
        //System.err.println("Expansion phase");
        for (int i = 0; i < forbiddenEdges.size(); i++) {
            Edge e = forbiddenEdges.get(i);
            forbiddenEdges.remove(e);
            List<Node> path = aStarSearch(vertices, initialCfg, goalCfg);
            // Significant edge found!
            if (path != null) {
                System.err.println("Significant edge found");
                significantEdge.add(e);
            }
                forbiddenEdges.add(e);
        }

        for (Edge significantEdge : significantEdge) {
            Point2D p1 = significantEdge.cfg1.getPosition(0);
            Point2D p2 = significantEdge.cfg2.getPosition(0);

            double w = p2.getX() - p1.getX();
            double h = p2.getY() - p1.getY();

            Rectangle2D r = new Rectangle2D.Double(p1.getX(), p1.getY(), w * 1.25, h * 1.25);
            vertices.addAll(new Sampler(problemSpec.getObstacles(), r).sampleUniformly(initialCfg.getASVCount(), CLASH_SAMPLES));
        }
        significantEdge.clear();

        vertices.addAll(new Sampler(problemSpec.getObstacles(), new Rectangle2D.Double(0,0,1,1)).sampleUniformly(initialCfg.getASVCount(), 10));
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


    public List<ASVConfig> convertNodes(List<Node> nodes) {
        List<ASVConfig> path = new ArrayList<>();

        for (int i = nodes.size() - 1; i >= 0; i--) {
            path.add(nodes.get(i).config);
        }
        return path;
    }


    /**
     * Returns the soultion path.
     *
     * @param path
     *      List of ASVConfigs to be converted into a path.
     *
     * @return ASV Configuration path.
     */
    public List<ASVConfig> getSolution(List<ASVConfig> path) {
        List<ASVConfig> finalPath = new ArrayList<>();

        // Create segments
        List<ASVConfig> newPath = new ArrayList<>();
        newPath.add(path.get(0));
        for (int j = 1; j < path.size(); j++) {
            ASVConfig c = path.get(j);
            ASVConfig cc = newPath.get(newPath.size() - 1);
            List<ASVConfig> segment = ASVConfig.createSegment(c, cc, SOLUTION_STEP_SIZE);

            if (!isValidSegment(segment)) {
                newPath.add(path.get(j - 1));
            } else {
                return null;
            }
        }
        newPath.add(path.get(path.size() - 1));

        for (int j = 0; j < newPath.size() - 1; j++) {
            ASVConfig c = newPath.get(j);
            ASVConfig cc = newPath.get(j + 1);

            List<ASVConfig> segment = ASVConfig.createSegment(c, cc, SOLUTION_STEP_SIZE);
            finalPath.addAll(segment);
        }
        return finalPath;
    }


    public Edge checkForClash(List<ASVConfig> path) {
        for (int i = 0; i < path.size() - 1; i++) {
            ASVConfig cfg1 = path.get(i);
            ASVConfig cfg2 = path.get(i + 1);
            List<ASVConfig> segment = ASVConfig.createSegment(cfg1, cfg2, SOLUTION_STEP_SIZE);
            if (!isValidSegment(segment)) {
                Edge e = new Edge(cfg1, cfg2);
                return e;
            }
        }
        return null;
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
        List<ASVConfig> asvPath;
        List<ASVConfig> vertices = new ArrayList<>();

        vertices.add(initialConfig);
        vertices.add(goalConfig);

        vertices.addAll(sampler.sampleUniformly(initialConfig.getASVCount(), SAMPLE_SIZE));
        while (true) {
            path = aStarSearch(vertices, initialConfig, goalConfig);
            if (path != null) {
                //System.err.println("Potential Path found");
                asvPath = convertNodes(path);
                Edge clash = checkForClash(asvPath);
                if (clash == null) {
                    System.out.println("Solution found!");
                    problemSpec.setPath(getSolution(asvPath));
                    problemSpec.saveSolution("testing.txt");
                    break;
                } else {
                    //System.err.println("Clash found");
                    forbiddenEdges.add(clash);
                }
            } else {
                //System.err.println("No Path found");
                expansionPhase(vertices, initialConfig, goalConfig);
            }
        }
    }
}
