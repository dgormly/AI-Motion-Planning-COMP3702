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
			problemSpec.loadProblem("testcases/3ASV.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}

		VisualHelper vh = new VisualHelper();
		SearchAgent agent = new SearchAgent(problemSpec);
		List<Point2D> sample = agent.samplePoints(20000, 0, 0, 1, 1);
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

		for (int i = 1; i < finalPath.size(); i++) {
			Point2D pos = finalPath.get(i);
			ASVConfig c = agent.moveASV(asvFinalPath.get(i - 1), pos);
			if (!tester.isValidConfig(c, agent.expandedList)) {
				c = agent.getBestConfig(c, agent.generateConfigs(c));
				for (int x = i; x < finalPath.size(); x++) {
					asvFinalPath.set(x, c);
				}
				vh.clearLinkedPoints();
				vh.addLinkedPoints(c.getASVPositions());
				vh.repaint();
				vh.waitKey();
			}
			asvFinalPath.set(i, c);
			//vh.addLinkedPoints(c.getASVPositions());
		}
//		List<ASVConfig> finalPath = new ArrayList<>();
//		for (int i = 0; i < asvPath.size() - 1; i++) {
//			ASVConfig initial = asvPath.get(i);
//			ASVConfig goal = asvPath.get(i + 1);
//			finalPath.addAll(agent.transform(initial, goal));
//		}

//		for (ASVConfig asvConfig : finalPath) {
//			vh.addLinkedPoints(asvConfig.getASVPositions());
//		}
		vh.repaint();
	}
}
