package visdebug;

import agent.SearchAgent;
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
	public static void main(String[] args) throws IOException {
		ProblemSpec problemSpec = new ProblemSpec();
		problemSpec.loadProblem("testcases/7ASV.txt");

		SearchAgent agent = new SearchAgent(problemSpec);

		// Create points
		List<Point2D> pointList = agent.distributePoints(1000, 0, 0, 1, 1);
		// Create obstacles
		List<Obstacle> obstacleList = problemSpec.getObstacles();
		List<Rectangle2D> rectangle2DList = new ArrayList<>();
		for (Obstacle o : obstacleList) {
			rectangle2DList.add(o.getRect());
		}

		// Test
		VisualHelper visualHelper = new VisualHelper();
		visualHelper.addPoints(pointList);
		visualHelper.addRectangles(rectangle2DList);
		visualHelper.repaint();
		
		// Wait for user key press
		visualHelper.waitKey();
		
		// Clear points, then draw linkedPoints
		visualHelper.clearPoints();
	//	visualHelper.addLinkedPoints(points);
	//	visualHelper.addLinkedPoints(points2);
		visualHelper.repaint();
	}

}
