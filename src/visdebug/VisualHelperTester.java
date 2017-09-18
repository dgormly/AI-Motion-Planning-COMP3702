package visdebug;

import agent.SearchAgent;
import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.lang.reflect.Array;
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
			problemSpec.loadProblem("testcases/3ASV-easy.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}

		SearchAgent agent = new SearchAgent(problemSpec);

		ASVConfig initialConfig = problemSpec.getInitialState();
		VisualHelper visualHelper = new VisualHelper();
		visualHelper.addLinkedPoints(initialConfig.getASVPositions());
		visualHelper.addPoints(Arrays.asList(initialConfig.getPosition(0)));
		visualHelper.repaint();
		visualHelper.waitKey();

		initialConfig = agent.rotateASV(initialConfig, 1, 90);
		visualHelper.addLinkedPoints(initialConfig.getASVPositions());
		visualHelper.repaint();

	}

}
