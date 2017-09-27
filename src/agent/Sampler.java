package agent;

import problem.ASVConfig;
import problem.Obstacle;

import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

public class Sampler {
    /*
    Sample near obstacles

        Sample a configuration q1 uniformly at random.
        Sample a configuration q2 from the set of all configurations within D distance from q1, uniformly at random.
        If q1 is in-collision & q2 is collision-free
        Add q2 as a vertex in the state q1 graph.
        Else if q1 is collision-free & q2 is in-collision
        Add q1 as a vertex in the state graph.
    */
    public List<ASVConfig> sample(SampleEnum sampleType, List<Rectangle2D> areaToSearch, List<Obstacle> obstacles) {
        return null;
    }

    /*
    Sample inside of passage

        Sample a configuration q1 uniformly at random.
        Sample a configuration q2 from the set of all configurations within D distance from q1, uniformly at random.
        If q1 & q2 are in-collision,
        Check if the middle configuration qm = 0.5*(q1+q2) is collision free.
        If qm is collision-free, add qm as a vertex in the state graph.
     */

    /*
    Using workspace information.

        Narrow passages in C-space are often caused by narrow passages in the workspace.
        Relax problem into planning for a point robot.
        Discretize the workspace into uniform grid.
        Choose a point r on the robot.
        Find a path π assuming the robot is the point r.
        à π: sequence of grid cells.
    */

    private List<Obstacle> obstacles;
    private Rectangle2D sampleArea;

    public Sampler(List<Obstacle> obstacles, Rectangle2D sampleArea) {
        this.obstacles = obstacles;
        this.sampleArea = sampleArea;

    }

    public List<ASVConfig> sampleUniformly(int numASVs, int sampleSize) {
        List<ASVConfig> sampleList = new ArrayList();
        List<Point2D> invalidPoints = new ArrayList<>();

        for (int i = 0; i < sampleSize; i++) {
            Point2D point;

            do {
                double xPoint = Math.abs(Math.random() * sampleArea.getWidth() + sampleArea.getX());
                double yPoint = Math.abs(Math.random() * sampleArea.getHeight() + sampleArea.getY());
                point = new Point2D.Double(xPoint, yPoint);
                if (!checkValidPoint(point)) {
                    invalidPoints.add(point);
                }
            } while (!checkValidPoint(point));

            List<ASVConfig> configs = ASVConfig.sampleConfigurations(point, numASVs, 1, obstacles);
            sampleList.addAll(configs);

            // Sample in passages using bad points.
            for (Point2D invalidPoint : invalidPoints) {
                for (Point2D point2D : invalidPoints) {
                    if (invalidPoint.equals(point2D)) {
                        continue;
                    }
                    double x = (invalidPoint.getX() + point2D.getX()) / 2;
                    double y = (invalidPoint.getY() + point2D.getX()) / 2;
                    Point2D p = new Point2D.Double(x,y);
                    if (checkValidPoint(p)) {
                        sampleList.addAll(ASVConfig.sampleConfigurations(p, numASVs, 1, obstacles));
                    }
                }
            }
        }
        return sampleList;
    }


    /**
     * Sample near obstacles
     *
     * @param sampleSize
     * @return
     */
    public List<ASVConfig> sampleNearObstacles(ASVConfig temp,int sampleSize, double diam) {
        List<ASVConfig> sampleList = new ArrayList();
        for (int i = 0; i < sampleSize; i++) {
            Point2D p;
            Point2D q;

            ASVConfig c;
            ASVConfig d;

            // Get first point.
            while (sampleList.size() < sampleSize) {
                double xPoint = Math.abs(Math.random() * sampleArea.getWidth() + sampleArea.getX());
                double yPoint = Math.abs(Math.random() * sampleArea.getHeight() + sampleArea.getY());
                p = new Point2D.Double(xPoint, yPoint);

                // Sample in given radius.
                Ellipse2D circle = new Ellipse2D.Double(p.getX() - (diam / 2), p.getY() - (diam / 2), diam, diam);
                do {
                    double xP2 = Math.abs(Math.random() * circle.getWidth() + circle.getX());
                    double yP2 = Math.abs(Math.random() * circle.getHeight() + circle.getY());
                    q = new Point2D.Double(xP2, yP2);
                } while (!circle.contains(p));


                if (!checkValidPoint(p) && checkValidPoint(q)) {
                    sampleList.addAll(ASVConfig.sampleConfigurations(q, temp.getASVCount(), 1, obstacles));
                } else if (checkValidPoint(p) && !checkValidPoint(q)) {
                    sampleList.addAll(ASVConfig.sampleConfigurations(p, temp.getASVCount(), 1, obstacles));
                }
            }
        }
        return sampleList;
    }


    public List<ASVConfig> sampleGrid(int rows, int columns, ASVConfig cfg) {
        List<ASVConfig> grid = new ArrayList<>();
        double width = 1 / columns;
        double height = 1 / rows;

        for (int i = 1; i < rows - 1; i++) {
            for (int j = 1; j < columns - 1; j++) {
                Point2D p = new Point2D.Double(j * width, i * height);
                if (!checkValidPoint(p)) {
                    continue;
                }
                grid.addAll(ASVConfig.sampleConfigurations(p, cfg.getASVCount(),1, obstacles));
            }
        }

        return grid;
    }


    /***
     * Checks if a given point collides with any obstacles.
     *
     * @param point
     *      checks if a point is in a valid position.
     * @return True if point is in an obstacle.
     */
    private boolean checkValidPoint(Point2D point) {
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
}
