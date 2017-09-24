package agent;

import problem.ASVConfig;
import problem.Obstacle;

import java.awt.geom.Rectangle2D;
import java.util.List;

public class Sample {
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
}
