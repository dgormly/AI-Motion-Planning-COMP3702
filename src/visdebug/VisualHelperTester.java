package visdebug;

import agent.SearchAgent;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class VisualHelperTester {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		ProblemSpec problemSpec = new ProblemSpec();
		try {
			problemSpec.loadProblem("testcases/3ASV-easy.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}

		SearchAgent agent = new SearchAgent(problemSpec);

		// Create obstacles
		List<Obstacle> obstacleList = problemSpec.getObstacles();
		List<Rectangle2D> rectangle2DList = new ArrayList<>();
		for (Obstacle o : obstacleList) {
			rectangle2DList.add(o.getRect());
		}

		VisualHelper visualHelper = new VisualHelper();
		// Test
		ASVConfig initialConfig = new ASVConfig(3, "0.150 0.225 0.150 0.275 0.200 0.275 \n" +
				"0.850 0.225 0.850 0.275 0.900 0.275");
		Point2D p1 = new Point2D.Double(0.25, 0.5);
		initialConfig = agent.moveASV(initialConfig, p1);
		visualHelper.addLinkedPoints(initialConfig.getASVPositions());

		Point2D p2 = new Point2D.Double(0.5, 0.5);
		Point2D p3 = new Point2D.Double(0.75, 0.5);
		List<Point2D> pointList = new ArrayList<>();
		pointList.add(p1);
		pointList.add(p2);
		pointList.add(p3);
		visualHelper.addPoints(pointList);

		visualHelper.repaint();
		visualHelper.waitKey();

		initialConfig = agent.moveASV(initialConfig, p2);
		visualHelper.addLinkedPoints(initialConfig.getASVPositions());
		visualHelper.repaint();


	}

}
