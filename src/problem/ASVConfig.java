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
	 *
	 * @param anchor
	 * @param angles
	 */
	public ASVConfig(Point2D anchor, double[] angles) {
		for (int i = 0; i < angles.length + 1; i++) {
			Point2D p = new Point2D.Double();
			asvPositions.add(p);
		}
		asvPositions.set(0, anchor);
		setAngles(angles);
		this.angles = angles;
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
		angles = getAngles();
	}

	/**
	 * Copy constructor.
	 *
	 * @param cfg
	 *            the configuration to copy.
	 */
	public ASVConfig(ASVConfig cfg) {
		asvPositions = cfg.getASVPositions();
		angles = getAngles();
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
	 * Normalises an angle to the range (-pi, pi]
	 *
	 * @param angle
	 *            the angle to normalise.
	 * @return the normalised angle.
	 */
	public static double normaliseAngle(double angle) {
		while (angle <= -Math.PI) {
			angle += 2 * Math.PI;
		}
		while (angle > Math.PI) {
			angle -= 2 * Math.PI;
		}
		return angle;
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
			angle = normaliseAngle(angle);
			angles[i] = angle;
		}
		this.angles = angles;
		return angles;
	}


	/**
	 * Sets the angles of each arm joint in the ASV.
	 *
	 * @param angles
	 * 		the angle to set each asv at.
	 */
	public void setAngles(double[] angles) {
		double currentAngle = 0;
		for (int i = 0; i < angles.length; i++) {
			currentAngle = i == 0 ? angles[i] : angles[i-1] + angles[i];
			Point2D a = this.getPosition(i);
			Point2D b = this.getPosition(i + 1);
			double newX = 0.05 * Math.cos(currentAngle) + a.getX();
			double newY = 0.05 * Math.sin(currentAngle) + a.getY();
			b.setLocation(newX, newY);
		}
		this.angles = angles;
	}



	/**
	 * Samples an area ASVConfigs for a given point.
	 *
	 * @param numberOfConfigs
	 * 			number of configurations to sample.
	 * @return List of valid configs from the sample spot.
	 */
	public static List<ASVConfig> sampleConfigurations(Point2D anchor,int numASV, int numberOfConfigs, List<Obstacle> obstacles) {
		List<ASVConfig> validConfigs = new ArrayList<>();
		Tester tester = new Tester();

		// Begins generating sample configurations.
		while (validConfigs.size() < numberOfConfigs) {
			// Generate random numbers.
			double[] rAngles = new double[numASV - 1];
			double range = 180 * (numASV - 3);
			for (int i = 1; i < numASV; i++) {
				double initialAngleDegrees = i == 1 ? Math.random() * 360 : Math.random() * 180;
				//range = i == 1 ? initialAngleDegrees : range - initialAngleDegrees;
				initialAngleDegrees = normaliseAngle(Math.toRadians(initialAngleDegrees));
				rAngles[i - 1] = initialAngleDegrees;
			}

			// Check if angles are a valid combination.
			ASVConfig tempASV = new ASVConfig(anchor, rAngles);
			if (tester.isValidConfig(tempASV, obstacles)) {
				validConfigs.add(tempASV);
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
	public static List<ASVConfig> createSegment(ASVConfig initialCfg, ASVConfig goalConfig, double stepSize) {
		List<ASVConfig> segment = new ArrayList<>();

		/* Setup rate of change for X and Y */
		double[] a = initialCfg.getAngles();
		double[] b = goalConfig.getAngles();
		double distance = initialCfg.maxDistance(goalConfig);
		int iterations = (int) Math.ceil(distance / stepSize);

		Point2D p1 = initialCfg.getPosition(0);
		Point2D p2 = goalConfig.getPosition(0);

		// Get difference in angle.
		double[] dAngle = new double[a.length];
		for (int i = 0; i < a.length; i++) {
			dAngle[i] = normaliseAngle(b[i] - a[i]);
		}

		// Get rate of change
		double[] cAngle = new double[a.length];
		for (int i = 0; i < a.length; i++) {
			cAngle[i] = dAngle[i] / iterations;
		}


		// Get anchor point.
		double pointDistanceX = p2.getX() - p1.getX();
		double pointDistanceY = p2.getY() - p1.getY();


		segment.add(initialCfg);
		for (int j = 1; j < iterations; j++) {
			/* Get new anchor point. */
			double[] tempAngle = new double[cAngle.length];
			for (int i = 0; i < cAngle.length; i++) {
				tempAngle[i] = j * cAngle[i] + initialCfg.getAngles()[i];
			}
			Point2D anchor = new Point2D.Double(j * (pointDistanceX / iterations) + p1.getX(), j * (pointDistanceY / iterations) + p1.getY());
			ASVConfig tempCfg = new ASVConfig(anchor, tempAngle);
			segment.add(tempCfg);
		}

		return segment;
	}


}
