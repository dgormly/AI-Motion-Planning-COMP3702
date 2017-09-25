package visdebug;

import agent.Node;
import agent.SearchAgent;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import sun.management.Agent;
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
			problemSpec.loadProblem("testcases/3ASV-easy.txt");
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
			vertices.addAll(agent.sampleStateGraph(initialConfig.getASVCount(), 10, new Rectangle2D.Double(0, 0, 1, 1)));
			path = agent.findPath(vertices, initialConfig, goalConfig);
			for (ASVConfig vertex : vertices) {
				vh.addLinkedPoints(vertex.getASVPositions());
			}
			vh.repaint();
			vh.waitKey();

		} while (path == null);
		vh.repaint();
	}
}
