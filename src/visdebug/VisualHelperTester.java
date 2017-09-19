package visdebug;

import agent.Node;
import agent.SearchAgent;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class VisualHelperTester {

	/**
	 * @param args
	 */
	public static void main(String[] args) throws IOException {
		ProblemSpec problemSpec = new ProblemSpec();
		try {
			problemSpec.loadProblem("testcases/3ASV-easy.txt");
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

		VisualHelper visualHelper = new VisualHelper();


//		List<ASVConfig> finalPath = agent.finalSolution(asvPath);
//		finalPath.forEach((a) -> {
//			visualHelper.addLinkedPoints(a.getASVPositions());
//		});
//		visualHelper.addPoints(path);
		visualHelper.repaint();

		List<ASVConfig> inValidConfigs = agent.findInvalidConfigs(asvPath, obstacles);

		inValidConfigs.forEach((e) -> {
			visualHelper.addLinkedPoints(e.getASVPositions());
		});

		obstacles.forEach(o -> {
			visualHelper.addRectangles(Arrays.asList(o.getRect()));
		});

		visualHelper.clearAll();
		obstacles.forEach(o -> {
			visualHelper.addRectangles(Arrays.asList(o.getRect()));
		});

		for (ASVConfig c : inValidConfigs) {
			int degrees = 0;
			while (!agent.checkValidConfig(c, obstacles)) {
				if (degrees == 360) {
					System.out.println("Gonna need to do more than rotate bud");
					System.exit(1);
				}
				c = agent.rotateASV(c, 1, 1);
				degrees++;
			}
			visualHelper.addLinkedPoints(c.getASVPositions());
		}

		List<ASVConfig> finalPath = agent.finalSolution(asvPath);
		problemSpec.setPath(finalPath);
		problemSpec.saveSolution("testing");

	}

}
