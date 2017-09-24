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
	}

	public ASVConfig(Point2D anchor, double[] angles, ASVConfig cfg) {
		asvPositions = cfg.getASVPositions();
		setAngles(angles);
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
	 *
	 * @return angles in ascending order.
	 */
	public double[] getAngles() {
		double[] angles = new double[getASVCount() - 1];
		for (int i = 0; i < getASVCount() - 1; i++) {
			Point2D a = asvPositions.get(i);
			Point2D b = asvPositions.get(i + 1);
			double x = b.getX() - a.getX();
			double y = b.getY() - a.getY();
			double angle = Math.atan2(y, x);
			if (i != 0) {
				angle -= angles[i - 1];
			}
			angles[i] = Math.toDegrees(angle);
		}
		return angles;
	}

	public void setAngles(double[] angles) {
		for (int i = 0; i < angles.length; i++) {
			double angle;
			if (i == 0) {
				 angle = angles[i];
			} else {
				angle = angles[i] - angles[i - 1];
			}
			angle = Math.toRadians(angle);
			Point2D a = this.getPosition(i);
			Point2D b = this.getPosition(i + 1);
			double newX = 0.05 * Math.cos(angle) + a.getX();
			double newY = 0.05 * Math.sin(angle) + a.getY();
			b.setLocation(newX, newY);
		}
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
//			ASVConfig newConfig = new ASVConfig(this);
//			double range = 180 + (this.getASVCount() - 3) * 180;
//			for (int i = 1; i < this.getASVCount(); i++) {
//				double initialAngleDegrees = Math.random() * range * 2 - range;
//				rotate( i, (int) initialAngleDegrees);
//				range -= initialAngleDegrees;
//			}
//			rotate(newConfig.getASVCount() - 1, (int) range);
//			if (tester.isValidConfig(newConfig, obstacles)) {
//				validConfigs.add(newConfig);
//			}
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
		int asvCount = initialCfg.getASVCount();

		/* Setup rate of change for X and Y */
		double pointDistance[] = new double[asvCount];
		double x[] = new double[asvCount];
		double y[] = new double[asvCount];
		double m[] = new double[asvCount];

		for (int i = 0; i < asvCount; i++) {
			Point2D p1 = initialCfg.getPosition(i);
			Point2D p2 = goalConfig.getPosition(i);

			pointDistance[i] = p1.distance(p2);
			x[i] = p2.getX() - p1.getX();
			y[i] = p2.getY() - p2.getY();
		}

		double maxLength = 0;

		// Get max distance.
		for (int i = 0; i < asvCount; i++) {
			if (pointDistance[i] > maxLength) {
				maxLength = pointDistance[i];
			}
		}

		int iterations = (int) (maxLength / 0.001);

		// Convert x[] and y[] to rates.
		for (int i = 0; i < asvCount; i++) {
			y[i] /= iterations;
			x[i] /= iterations;
		}


		/* Loop */
		List<ASVConfig> segment = new ArrayList<>();
		segment.add(initialCfg);
		for (int i = 0; i < iterations; i++) {
			ASVConfig cfg = new ASVConfig(segment.get(i));
			ASVConfig prev = segment.get(i);

			for (int j = 0; j < asvCount; j++) {
				Point2D newSpot = new Point2D.Double(x[j] + prev.getPosition(j).getX(), y[j] + prev.getPosition(j).getY());
				cfg.getPosition(j).setLocation(newSpot);
			}
			segment.add(cfg);
		}
		return segment;
	}

	@Override
	public boolean equals(Object o) {

		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;

		ASVConfig asvConfig = (ASVConfig) o;

		return asvPositions != null ? asvPositions.equals(asvConfig.asvPositions) : asvConfig.asvPositions == null;
	}

	@Override
	public int hashCode() {
		return asvPositions != null ? asvPositions.hashCode() : 0;
	}
}
