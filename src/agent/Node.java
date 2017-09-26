package agent;

import problem.ASVConfig;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class Node implements Comparable<Node> {
    public Node parent;
    public List<Node> children;
    public double pathCost;
    public ASVConfig config;

    public Node(Node parent, double cost, ASVConfig config) {
        this.parent = parent;
        this.children = new ArrayList<>();
        this.pathCost = cost;
        this.config = config;
    }

    public Node(Node parent, ASVConfig config) {
        this.parent = parent;
        this.children = new ArrayList<>();
        this.config = config;
    }

    public Node() {

    }

    public int compareTo(Node o) {
        return Double.compare(this.pathCost, o.pathCost);
    }

}
