package agent;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class Node implements Comparable<Node> {
    public Node parent;
    public List<Node> children;
    public Point2D point;
    public double pathCost;

    public Node(Node parent, Point2D point, double cost) {
        this.parent = parent;
        this.children = new ArrayList<>();
        this.point = point;
        this.pathCost = cost;
    }

    public int compareTo(Node o) {
        return Double.compare(this.pathCost, o.pathCost);
    }

}
