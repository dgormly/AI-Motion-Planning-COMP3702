package visdebug;

import agent.SearchAgent;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
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
		do {
			visualHelper.clearAll();
			List<Point2D> vertices = agent.samplePoints(500, 0, 0, 1, 1);
			visualHelper.addRectangles(rectangle2DList);
			visualHelper.addPoints(vertices);
			visualHelper.repaint();
			visualHelper.waitKey();
			//agent.createGraph(vertices).forEach((v) -> visualHelper.addLinkedPoints(v));
			Point2D p1 = new Point2D.Double(0.2, 0.1);
			Point2D p2 = new Point2D.Double(0.8, 0.9);
			visualHelper.addLinkedPoints(agent.findPath(vertices, p1, p2));
			visualHelper.repaint();
			visualHelper.waitKey();
		} while (true);
	}

}
