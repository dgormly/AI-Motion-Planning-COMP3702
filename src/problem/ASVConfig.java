package problem;

import tester.Tester;

import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.*;
import java.awt.geom.Point2D;

/**
 * Represents a configuration of the ASVs. This class doesn't do any validity
 * checking - see the code in tester.Tester for this.
 *
 * @author lackofcheese
 */
public class ASVConfig {
	/** The position of each ASV */
	private List<Point2D> asvPositions = new ArrayList<Point2D>();
	private double[] angles;

	/**
	 * Constructor. Takes an array of 2n x and y coordinates, where n is the
	 * number of ASVs
	 *
	 * @param coords
	 *            the x- and y-coordinates of the ASVs.
	 */
	public ASVConfig(double[] coords) {
		for (int i = 0; i < coords.length / 2; i++) {
			asvPositions.add(new Point2D.Double(coords[i * 2],
					coords[i * 2 + 1]));
		}
		angles = getAngles();
	}

	/**
	 * Constructs an ASVConfig from a space-separated string of x- and y-
	 * coordinates
	 *
	 * @param asvCount
	 *            the number of ASVs to read.
	 * @param str
	 *            the String containing the coordinates.
	 */
	public ASVConfig(int asvCount, String str) throws InputMismatchException {
		Scanner s = new Scanner(str);
		for (int i = 0; i < asvCount; i++) {
			asvPositions
					.add(new Point2D.Double(s.nextDouble(), s.nextDouble()));
		}
		s.close();
	}

	/**
	 * Copy constructor.
	 *
	 * @param cfg
	 *            the configuration to copy.
	 */
	public ASVConfig(ASVConfig cfg) {
		asvPositions = cfg.getASVPositions();
	}

	/**
	 * Returns a space-separated string of the ASV coordinates.
	 *
	 * @return a space-separated string of the ASV coordinates.
	 */
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for (Point2D point : asvPositions) {
			if (sb.length() > 0) {
				sb.append(" ");
			}
			sb.append(point.getX());
			sb.append(" ");
			sb.append(point.getY());
		}
		return sb.toString();
	}

	/**
	 * Returns the maximum straight-line distance between the ASVs in this state
	 * vs. the other state, or -1 if the ASV counts don't match.
	 *
	 * @param otherState
	 *            the other state to compare.
	 * @return the maximum straight-line distance for any ASV.
	 */
	public double maxDistance(ASVConfig otherState) {
		if (this.getASVCount() != otherState.getASVCount()) {
			return -1;
		}
		double maxDistance = 0;
		for (int i = 0; i < this.getASVCount(); i++) {
			double distance = this.getPosition(i).distance(
					otherState.getPosition(i));
			if (distance > maxDistance) {
				maxDistance = distance;
			}
		}
		return maxDistance;
	}

	/**
	 * Returns the total straight-line distance over all the ASVs between this
	 * state and the other state, or -1 if the ASV counts don't match.
	 *
	 * @param otherState
	 *            the other state to compare.
	 * @return the total straight-line distance over all ASVs.
	 */
	public double totalDistance(ASVConfig otherState) {
		if (this.getASVCount() != otherState.getASVCount()) {
			return -1;
		}
		double totalDistance = 0;
		for (int i = 0; i < this.getASVCount(); i++) {
			totalDistance += this.getPosition(i).distance(
					otherState.getPosition(i));
		}
		return totalDistance;
	}

	/**
	 * Returns the position of the ASV with the given number.
	 *
	 * @param asvNo
	 *            the number of the ASV.
	 * @return the position of the ASV with the given number.
	 */
	public Point2D getPosition(int asvNo) {
		return asvPositions.get(asvNo);
	}

	/**
	 * Returns the number of ASVs in this configuration.
	 *
	 * @return the number of ASVs in this configuration.
	 */
	public int getASVCount() {
		return asvPositions.size();
	}

	/**
	 * Returns the positions of all the ASVs, in order.
	 *
	 * @return the positions of all the ASVs, in order.
	 */
	public List<Point2D> getASVPositions() {
		List<Point2D> newList = new ArrayList();
		for (Point2D p : asvPositions) {
			Point2D tempPoint = (Point2D) p.clone();
			newList.add(tempPoint);
		}
		return newList;
	}


	/**
	 * Get all the angles from the config.
	 * THIS HASN"T BEEN TESTED!
	 * @return angles in ascending order.
	 */
	public double[] getAngles() {
		double[] angles = new double[getASVCount() - 1];
		for (int i = 1; i < asvPositions.size(); i++) {
			Point2D originalPoint = asvPositions.get(i - 1);
			Point2D p = asvPositions.get(i);
			double x = p.getX() - originalPoint.getX();
			double y = p.getY() - originalPoint.getY();
			double angle = Math.atan2(y, x);
			if (i != 1) {
				angle += angles[i - 1];
			}
			angles[i] = angle;
		}
		return angles;
	}


	/**
	 * Rotates an arm around a join at a diameter of 0.001.
	 *
	 * @param anchorPoint
	 * 			Joint to rotate.
	 * @param point
	 * 			Point to rotate around join.
	 * @param degree
	 * 			Amount of degrees to rotate arm.
	 */
	private void rotatePoint(int anchorPoint, int point, double degree) {
		Point2D p1 = this.getPosition(anchorPoint);
		Point2D p2 = this.getPosition(point);

		double h = p1.distance(p2);
		double rad = Math.toRadians(degree);
		double changeX = p2.getX() - p1.getX();
		double changeY = p2.getY() - p1.getY();
		double currentRad = Math.atan2(changeY, changeX);

		// New Pos
		double newX = h * Math.cos(rad + currentRad) + p1.getX();
		double newY = h * Math.sin(currentRad + rad) + p1.getY();
		p2.setLocation(newX, newY);
	}

	/**
	 * Rotates the whole configuration from a particular joint.
	 *
	 * @param pointNumber
	 * 		The point to rotate. 1 Rotates everything.
	 * @param degrees
	 * 		rotates the point between [0 - 360]
	 */
	public void rotate(int pointNumber, double degrees) {
		for (int i = pointNumber; i < this.getASVCount(); i++) {
			rotatePoint(i - 1, i, degrees);
		}
	}


	/**
	 * Moves the configuring to a given point.
	 *
	 * @param newPos
	 * 		The point to place the configuration.
	 */
	public void move(Point2D newPos) {
		Point2D anchorPoint = this.getPosition(0);
		// Configure asvs relative to parent.
		for (int i = 1; i < this.getASVCount(); i++) {
			Point2D p = this.getPosition(i);
			double differenceX = p.getX() - anchorPoint.getX();
			double differenceY = p.getY() - anchorPoint.getY();
			p.setLocation(newPos.getX() + differenceX, newPos.getY() + differenceY);
		}
		this.getPosition(0).setLocation(newPos);
	}

	/**
	 * Samples an area ASVConfigs for a given point.
	 *
	 * @param numberOfConfigs
	 * 			number of configurations to sample.
	 * @return List of valid configs from the sample spot.
	 */
	public List<ASVConfig> sampleConfigurations(int numberOfConfigs, List<Obstacle> obstacles) {
		List<ASVConfig> validConfigs = new ArrayList<>();
		Tester tester = new Tester();

		// Begins generating sample configurations.
		while (validConfigs.size() < numberOfConfigs) {
			ASVConfig newConfig = new ASVConfig(this);
			double range = 180 + (this.getASVCount() - 3) * 180;
			for (int i = 1; i < this.getASVCount(); i++) {
				double initialAngleDegrees = Math.random() * range * 2 - range;
				rotate( i, (int) initialAngleDegrees);
				range -= initialAngleDegrees;
			}
			rotate(newConfig.getASVCount() - 1, (int) range);
			if (tester.isValidConfig(newConfig, obstacles)) {
				validConfigs.add(newConfig);
			}
		}
		return validConfigs;
	}

	/**
	 * TO FINISH.
	 * Returns the configuration that has the least distance.
	 *
	 * @param config
	 * @param list
	 * @return
	 */
	public static ASVConfig getBestConfig(ASVConfig config, List<ASVConfig> list) {
		double dist = 2;
		ASVConfig best = new ASVConfig(list.get(0));
		for (ASVConfig asvConfig : list) {
			double tempDist = config.totalDistance(asvConfig);
			if (tempDist < dist) {
				dist = tempDist;
			}
		}
		return best;
	}


	/**
	 *  Returns a list of configurations from one config to another.
	 *
	 * @param initialCfg
	 * @param goalConfig
	 * @return Segment between two configurations.
	 */
	public static List<ASVConfig> transform(ASVConfig initialCfg, ASVConfig goalConfig) {
		List<ASVConfig> finalSolution = new ArrayList<>();
		finalSolution.add(initialCfg);
		while (true) {
			ASVConfig cfg = new ASVConfig(finalSolution.get(finalSolution.size() - 1));
			for (int i = 0; i < initialCfg.getASVCount(); i++) {
				double yDist = goalConfig.getPosition(i).getY() - cfg.getPosition(i).getY();
				double xDist = goalConfig.getPosition(i).getX() - cfg.getPosition(i).getX();
				double angle = Math.atan2(yDist , xDist);
				double distance = goalConfig.getPosition(i).distance(cfg.getPosition(i));
				double stepDist = distance > 0.001 ? 0.001 : Math.abs(distance);

				double xRate = stepDist * Math.cos(angle);
				double yRate = stepDist * Math.sin(angle);

				double x = cfg.getPosition(i).getX() + xRate;
				double y = cfg.getPosition(i).getY() + yRate;
				cfg.getPosition(i).setLocation(x, y);
				if (cfg.totalDistance(goalConfig) < 0.001) {
					return finalSolution;
				}
			}
			finalSolution.add(cfg);
		}
	}


}
