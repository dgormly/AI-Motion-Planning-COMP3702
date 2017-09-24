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
			problemSpec.loadProblem("testcases/3ASV-easy.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}

		VisualHelper vh = new VisualHelper();
		SearchAgent agent = new SearchAgent(problemSpec);
		List<Point2D> sample = agent.samplePoints(2000, 0, 0, 1, 1);
		ASVConfig initialConfig = problemSpec.getInitialState();
		ASVConfig goalConfig = problemSpec.getGoalState();
		Tester tester = new Tester();
		List<Node> path = agent.findPath(sample, initialConfig, goalConfig);
		List<ASVConfig> asvPath = new ArrayList<>();
		List<Obstacle> obstacleList = problemSpec.getObstacles();
		List<Rectangle2D> rectList = new ArrayList<>();
		List<Point2D> pointPath = new ArrayList<>();
		for (Obstacle obstacle : obstacleList) {
			rectList.add(obstacle.getRect());
		}
		for (Node node : path) {
			asvPath.add(node.config);
			pointPath.add(node.point);
		}

		vh.addRectangles(rectList);
		vh.repaint();
		vh.waitKey();
		vh.addPoints(sample);
		vh.repaint();
		vh.waitKey();

		List<Point2D> finalPath = new ArrayList<>();
		for (int i = 0; i < pointPath.size() - 1; i++) {
			Point2D p = pointPath.get(i);
			Point2D pp = pointPath.get(i + 1);
			finalPath.addAll(agent.joinPoints(p,pp));
		}

		vh.clearPoints();
		vh.addLinkedPoints(finalPath);
		vh.repaint();
		vh.waitKey();

		List<ASVConfig> asvFinalPath = new ArrayList<>();
		// Load array
		for (int i = 0; i < finalPath.size(); i++) {
			asvFinalPath.add(new ASVConfig(initialConfig));
		}
		vh.clearLinkedPoints();
		vh.repaint();
		vh.waitKey();
	}
}
