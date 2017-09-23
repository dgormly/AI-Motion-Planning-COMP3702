package agent;

import problem.ASVConfig;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class Node implements Comparable<Node> {
    public Node parent;
    public List<Node> children;
    public Point2D point;
    public double pathCost;
    public ASVConfig config;

    public Node(Node parent, Point2D point, double cost, ASVConfig config) {
        this.parent = parent;
        this.children = new ArrayList<>();
        this.point = point;
        this.pathCost = cost;
        this.config = config;
    }

    public Node(Node parent, Point2D point, ASVConfig config) {
        this.parent = parent;
        this.children = new ArrayList<>();
        this.point = point;
        this.config = config;
    }

    public int compareTo(Node o) {
        return Double.compare(this.pathCost, o.pathCost);
    }

}
