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
			problemSpec.loadProblem("testcases/7ASV.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}

		SearchAgent agent = new SearchAgent(problemSpec);
		List<Obstacle> obstacles = problemSpec.getObstacles();

		ASVConfig initialConfig = problemSpec.getInitialState();
		ASVConfig finalConfig = problemSpec.getGoalState();

		VisualHelper vh = new VisualHelper();
		List<ASVConfig> configs = agent.generateConfigs(initialConfig);
		configs.forEach(c -> {
			vh.addLinkedPoints(c.getASVPositions());
		});
		vh.repaint();
		vh.waitKey();
		vh.clearAll();
		vh.addPoints(agent.getBestConfig(initialConfig, configs).getASVPositions());
		vh.repaint();
		vh.waitKey();
	}

}
