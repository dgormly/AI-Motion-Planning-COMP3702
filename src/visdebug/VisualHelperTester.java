package visdebug;

import agent.Node;
import agent.Sample;
import agent.SearchAgent;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import sun.management.Agent;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class VisualHelperTester {

	/**
	 * @param args
	 */
	public static void main(String[] args) throws IOException, InterruptedException {
		ProblemSpec problemSpec = new ProblemSpec();
		problemSpec.assumeDirectSolution();
		try {
			problemSpec.loadProblem("testcases/3ASV.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}
		SearchAgent agent = new SearchAgent(problemSpec);
		VisualHelper vh = new VisualHelper();
		ASVConfig initialConfig = problemSpec.getInitialState();
		ASVConfig goalConfig = problemSpec.getGoalState();
		List<Rectangle2D> rectangles = new ArrayList<>();
		for (Obstacle obstacle : problemSpec.getObstacles()) {
			rectangles.add(obstacle.getRect());
		}
		vh.addRectangles(rectangles);

		List<Node> path;
		List<ASVConfig> vertices = new ArrayList<>();

		do {
			vh.clearLinkedPoints();
			List<ASVConfig> ran = new Sample(problemSpec.getObstacles(), new Rectangle2D.Double(0,0,1,1)).sampleInsidePassage(initialConfig, 10,0.4);
			for (ASVConfig asvConfig : ran) {
				vh.addLinkedPoints(asvConfig.getASVPositions());
			}
			vh.repaint();
			vh.waitKey();

		} while (true);
	}
}
