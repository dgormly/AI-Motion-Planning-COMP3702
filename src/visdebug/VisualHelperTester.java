package visdebug;

import agent.Node;
import agent.SearchAgent;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class VisualHelperTester {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		ProblemSpec problemSpec = new ProblemSpec();
		try {
			problemSpec.loadProblem("testcases/3ASV.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}

		SearchAgent agent = new SearchAgent(problemSpec);
		List<Obstacle> obstacles = problemSpec.getObstacles();

		ASVConfig initialConfig = problemSpec.getInitialState();
		ASVConfig finalConfig = problemSpec.getGoalState();
		List<Point2D> sample = agent.samplePoints(5000, 0,0,1,1);
		List<Node> nodePath = agent.findPath(sample, initialConfig, finalConfig);
		List<Point2D> path = new ArrayList<>();
		List<ASVConfig> asvPath = new ArrayList<>();
		nodePath.forEach(n -> {
			asvPath.add(n.config);
			path.add(n.point);
		});

		List<ASVConfig> finalPath = agent.finalSolution(asvPath);

//		List<ASVConfig> inValidConfigs = agent.findInvalidConfigs(initialConfig, path, obstacles);
//		for (ASVConfig c : inValidConfigs) {
//			for (int i = 1; i < c.getASVPositions().size(); i++) {
//				Point2D a = c.getPosition(0);
//				Point2D p = c.getPosition(i);
//				while (!agent.checkValidPoint(p)) {
//					p = agent.rotatePoint(a, p, -1);
//				}
//			}
//		}

		VisualHelper visualHelper = new VisualHelper();
		finalPath.forEach((a) -> {
			visualHelper.addLinkedPoints(a.getASVPositions());
		});
		obstacles.forEach(o -> {
			visualHelper.addRectangles(Arrays.asList(o.getRect()));
		});
		visualHelper.addPoints(path);
		visualHelper.repaint();

	}

}
