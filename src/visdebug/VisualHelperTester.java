package visdebug;

import agent.SearchAgent;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

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
			problemSpec.loadProblem("testcases/7ASV-easy.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}

		VisualHelper vh = new VisualHelper();
		SearchAgent agent = new SearchAgent(problemSpec);
		ASVConfig initialConfig = problemSpec.getInitialState();
		ASVConfig goalConfig = problemSpec.getGoalState();
		Tester tester = new Tester();
		List<ASVConfig> asvPath = new ArrayList<>();
		List<Obstacle> obstacleList = problemSpec.getObstacles();
		List<Rectangle2D> rectList = new ArrayList<>();


	vh.addLinkedPoints(goalConfig.getASVPositions());
	vh.repaint();
	vh.waitKey();
		List<ASVConfig> path = ASVConfig.transform(initialConfig, goalConfig);

		vh.addLinkedPoints(goalConfig.getASVPositions());
		double[] results = goalConfig.getAngles();
		vh.repaint();

	}
}
