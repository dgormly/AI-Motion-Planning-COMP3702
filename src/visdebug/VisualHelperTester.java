package visdebug;

import agent.Node;
import agent.SearchAgent;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import sun.management.resources.agent;
import tester.Tester;

import java.awt.geom.Line2D;
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
		try {
			problemSpec.loadProblem("testcases/7ASV.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}

		VisualHelper vh = new VisualHelper();
		SearchAgent agent = new SearchAgent(problemSpec);
		List<Point2D> sample = agent.samplePoints(2000, 0, 0, 1, 1);
		ASVConfig initialConfig = problemSpec.getInitialState();
		ASVConfig goalConfig = problemSpec.getGoalState();
		Tester tester = new Tester();

		List<ASVConfig> asvPath = agent.transform(initialConfig, goalConfig);
		for (ASVConfig asvConfig : asvPath) {
			vh.addLinkedPoints(asvConfig.getASVPositions());
		}
		vh.repaint();
		vh.waitKey();
	}

}
