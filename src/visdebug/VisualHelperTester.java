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
			problemSpec.loadProblem("testcases/3ASV-easy.txt");
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

		List<ASVConfig> path = ASVConfig.transform(initialConfig, goalConfig);
		tester.getInvalidBoomStates();
	}
}
