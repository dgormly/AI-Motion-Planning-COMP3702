package agent;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class Node {
    public Node parent;
    public List<Node> children;
    public Point2D point;

    public Node(Node parent, Point2D point) {
        this.parent = parent;
        this.children = new ArrayList<>();
        this.point = point;
    }
}
