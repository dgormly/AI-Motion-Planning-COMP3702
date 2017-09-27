package visdebug;

import agent.Edge;
import agent.Node;
import agent.Sampler;
import agent.SearchAgent;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
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
		List<ASVConfig> asvPath;
		List<ASVConfig> vertices = new ArrayList<>();
		Sampler sampler = new Sampler(problemSpec.getObstacles(), new Rectangle2D.Double(0, 0, 1, 1));
		vertices.addAll(sampler.sampleUniformly(initialConfig.getASVCount(), 5));

		while (true) {
			path = agent.aStarSearch(vertices, initialConfig, goalConfig);
			if (path != null) {
				asvPath = agent.convertNodes(path);

				// Print
				List<Point2D> pPath = new ArrayList<>();
				for (ASVConfig asvConfig : asvPath) {
					vh.addLinkedPoints(asvConfig.getASVPositions());
					pPath.add(asvConfig.getPosition(0));
					vh.addLinkedPoints(pPath);
				}
				Edge clash = agent.checkForClash(asvPath);
				if (clash == null) {
					System.out.println("Solution found!");
					System.exit(0);
				} else {
					agent.forbiddenEdges.add(clash);
					vertices.remove(clash);
				}
			} else {
				agent.expansionPhase(vertices, initialConfig, goalConfig);
			}

		}
	}
}
