package ca.mcgill.ecse211.mountev3rest.navigation;

import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import ca.mcgill.ecse211.mountev3rest.sensor.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Provides localization functionality, which allows the robot to compute its initial location when
 * placed on the grid.
 * <p>
 * The {@code Localizator} class combines ultrasonic and light sensor localization into a single
 * routine that corrects the X, Y and Theta values of the odometer to the location of the robot with
 * respect to the grid.
 * 
 * @author angelortiz
 *
 */
public class Localizer {

  // Constants
  private static final int LOCALIZATION_PERIOD = 25;
  private static final int ROTATE_SPEED = 80;
  private static final int THRESHOLD = 45;
  private static final double HALF_TILE_APPROX = 0.5;
  private final double SENSOR_OFFSET;
  private final double SENSOR_OFFSET_ANGLE;
  private final double TILE_SIZE;

  // Object Attributes
  private Odometer odometer;
  private OdometryCorrector corrector;
  private UltrasonicPoller usPoller;
  private LightPoller lightPoller;
  private Navigation navigation;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  // Localization Attributes
  private int prevDistance;
  private int currDistance;
  private boolean measurementTaken;
  private double alpha;
  private double beta;

  /**
   * Creates a {@code Localizator} that will operate on the singleton {@code Odometer} class.
   * 
   * @param leftMotor Lower left motor of the robot.
   * @param rightMotor Lower right motor of the robot.
   * @param navigation Navigation object to move the robot on the grid.
   * @param SENSOR_OFFSET Distance between the lower light sensor and the robot's center.
   * @param SENSOR_OFFSET_ANGLE Angle between the lower light sensor and the robot's center.
   * @param TILE_SIZE Size of a tile in the grid in centimeters.
   * 
   * @throws OdometerException If the {@code Odometer} has not been instantiated.
   * @throws PollerException If the {@code UltrasonicPoller} of the {@code LightPoller} classes have
   *         not been instantiated.
   * @see Odometer
   */
  public Localizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      Navigation navigation, OdometryCorrector corrector, final double SENSOR_OFFSET, final double SENSOR_OFFSET_ANGLE,
      final double TILE_SIZE) throws OdometerException, PollerException {
    usPoller = UltrasonicPoller.getUltrasonicPoller();
    lightPoller = LightPoller.getLightPoller();
    odometer = Odometer.getOdometer();
    this.corrector = corrector;

    this.navigation = navigation;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.SENSOR_OFFSET = SENSOR_OFFSET;
    this.SENSOR_OFFSET_ANGLE = SENSOR_OFFSET_ANGLE;
    this.TILE_SIZE = TILE_SIZE;
  }

  /**
   * Performs falling edge ultrasonic localization to correct the angle Theta of the odometer and
   * then it uses light localization to get the correct values of the X and Y coordinates of the
   * robot's location. The values obtained are used to overwrite the odometer's current values.
   * <p>
   * The starting corner parameter is used to adjust the odometer values according the the absolute
   * set of coordinated on the map. The corners are defined using the range {@code [0-3]} with 0
   * being the lower left corner and the range increasing in the counter-clockwise direction.
   * 
   * @param startingCorner Starting corner of the robot on the grid.
   * 
   */
  public void localize(int startingCorner, int xLim, int yLim) {
    ultrasonicLocalization(startingCorner);

    // Determine the reference point for light localization based on the starting corner.
    int refX = 0, refY = 0;
    switch(startingCorner) {
      case 0:
        refX = 1;
        refY = 1;
        break;
      case 1:
        refX = xLim - 1;
        refY = 1;
        break;
      case 2:
        refX = xLim - 1;
        refY = yLim - 1;
        break;
      case 3:
        refX = 1;
        refY = yLim - 1;
        break;
    }
    
    lightLocalization(startingCorner, refX, refY);
    navigation.setLocation(refX, refY);
  }

  /**
   * Estimates the angle of the robot with respect to the grid by rotating until a falling edge is
   * detected with the wall on each side. The angle in the middle of the two falling edge detections
   * is used to estimate the robot's real angle.
   * 
   * @param startingCorner Starting corner of the robot on the grid.
   * 
   */
  private void ultrasonicLocalization(int startingCorner) {
    long updateStart, updateEnd;
    prevDistance = -1;
    boolean firstSearch = true;

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();

    // Falling edge localization
    while (true) {
      updateStart = System.currentTimeMillis();
      currDistance = usPoller.poll();
      if (prevDistance < 0) {
        prevDistance = currDistance;
        continue;
      }
      if (prevDistance >= currDistance && prevDistance > THRESHOLD) { // Distance increasing
        if (!measurementTaken && currDistance < THRESHOLD) {
          if (firstSearch) {
            beta = odometer.getXYT()[2];
            leftMotor.backward();
            rightMotor.forward();
            firstSearch = false;
            measurementTaken = true;
          } else {
            alpha = odometer.getXYT()[2];
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            break;
          }
        }
      } else if (prevDistance < currDistance) { // Distance decreasing
        measurementTaken = false;
      }
      prevDistance = currDistance;
      // This ensures that the navigator only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < LOCALIZATION_PERIOD) {
        try {
          Thread.sleep(LOCALIZATION_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
    correctAngle();
    
    // Adjust to starting position
    switch (startingCorner) {
      case 0:
        break;
      case 1:
        odometer.update(0, 0, 270);
        break;
      case 2:
        odometer.update(0, 0, 180);
        break;
      case 3:
        odometer.update(0, 0, 90);
        break;
    }
    
  }

  /**
   * Provides a more accurate set of values for X, Y and Theta to the odometer by using light
   * localization.
   * <p>
   * The light localization routine consists of approaching the reference corner and spinning until
   * four line detections are recorded. The the angles recorded during the line detections are used
   * to correct the odometer values.
   * <p>
   * This method requires that the robot is placed around the middle of a tile and that the odometer
   * knows the angle of the robot to some degree of accuracy. A reference corner and its coordinates
   * must be provided to use to do the localization. The corners are encoded using the range
   * {@code [0-3]} with 0 being the the upper right corner of the tile where the robot lies, and the
   * range increasing in the counter-clockwise direction.
   * 
   * @param referenceCorner Reference corner to use for line localization subroutine.
   * @param refX X coordinate of the reference corner.
   * @param refY Y coordinate of the reference corner.
   */
  public void lightLocalization(int referenceCorner, int refX, int refY) {
    boolean inLine = false;
    int lineDetections = 0;
    double[] angleDetectionRaw = new double[4]; // Array to store the angle detections
    double[] angleDetection = new double[4];
    int approxAngle = -1;

    // Assume the robot is around the middle of the tile, move towards the closest corner.
    switch (referenceCorner) {
      case 0:
        approxAngle = 45;
        break;
      case 1:
        approxAngle = 315;
        break;
      case 2:
        approxAngle = 225;
        break;
      case 3:
        approxAngle = 135;
        break;
    }

    boolean wasEnabled = corrector.isEnabled();
    corrector.disable();
    
    navigation.turnTo(approxAngle);

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(
        Navigation.convertDistance(navigation.WHEEL_RADIUS, TILE_SIZE * HALF_TILE_APPROX), true);
    rightMotor.rotate(
        Navigation.convertDistance(navigation.WHEEL_RADIUS, TILE_SIZE * HALF_TILE_APPROX), false);

    // Rotate counter-clockwise and record 4 angles at line detections. 205, 110
    leftMotor.backward();
    rightMotor.forward();

    while (lineDetections < 4) {
      if (lightPoller.inLine) {
        if (!inLine) {
          Sound.beep();
          angleDetectionRaw[lineDetections] = odometer.pollGyro();
          angleDetection[lineDetections] = odometer.getXYT()[2];
          lineDetections++;
          inLine = true;
        }
      } else {
        inLine = false;
      }
    }

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

    // Correct the odometer using the angles of the line detections.
    double nX = angleDetectionRaw[3]; // Negative X intersection
    double pX = angleDetectionRaw[1]; // Positive X intersection
    double nY = angleDetectionRaw[0]; // Negative Y intersection
    double pY = angleDetectionRaw[2]; // Positive Y intersection

    double thetaY = pY - nY;
    double thetaX = nX - pX;

    double correctedX = 0;
    double correctedY = 0;
    double thetaXError = 0;
    double thetaYError = 0;
    double averageThetaError = 0;
    
    switch (referenceCorner) {
      case 0:
        correctedX = TILE_SIZE * refX - SENSOR_OFFSET * Math.cos(Math.toRadians(thetaY) / 2);
        correctedY = TILE_SIZE * refY - SENSOR_OFFSET * Math.cos(Math.toRadians(thetaX) / 2);
        thetaXError = 180 - (thetaX / 2) + SENSOR_OFFSET_ANGLE - angleDetection[3];
        thetaYError = 270 - (thetaY / 2) + SENSOR_OFFSET_ANGLE - angleDetection[2];
        averageThetaError = (thetaXError + thetaYError) / 2;
        break;
      case 1:
        correctedY = TILE_SIZE * refY - SENSOR_OFFSET * Math.cos(Math.toRadians(thetaY) / 2);
        correctedX = TILE_SIZE * refX + SENSOR_OFFSET * Math.cos(Math.toRadians(thetaX) / 2); 
        thetaXError = 90 - (thetaX / 2) + SENSOR_OFFSET_ANGLE - angleDetection[3];
        thetaYError = 180 - (thetaY / 2) + SENSOR_OFFSET_ANGLE - angleDetection[2];
        averageThetaError = (thetaXError + thetaYError) / 2;
        break;
      case 2:
        correctedX = TILE_SIZE * refX + SENSOR_OFFSET * Math.cos(Math.toRadians(thetaY) / 2);
        correctedY = TILE_SIZE * refY + SENSOR_OFFSET * Math.cos(Math.toRadians(thetaX) / 2);
        thetaXError = 360 - (thetaX / 2) + SENSOR_OFFSET_ANGLE - angleDetection[3];
        thetaYError = 90 - (thetaY / 2) + SENSOR_OFFSET_ANGLE - angleDetection[2];
        averageThetaError = (thetaXError + thetaYError) / 2;
        break;
      case 3:
        correctedY = TILE_SIZE * refY + SENSOR_OFFSET * Math.cos(Math.toRadians(thetaY) / 2);
        correctedX = TILE_SIZE * refX - SENSOR_OFFSET * Math.cos(Math.toRadians(thetaX) / 2);
        thetaXError = 270 - (thetaX / 2) + SENSOR_OFFSET_ANGLE - angleDetection[3];
        thetaYError = 360 - (thetaY / 2) + SENSOR_OFFSET_ANGLE - angleDetection[2];
        averageThetaError = (thetaXError + thetaYError) / 2;
        break;
    }

    odometer.setXYT(correctedX, correctedY, angleDetection[3]);
    odometer.update(0, 0, averageThetaError);
    
    if (wasEnabled)
      corrector.enable();
  }

  /**
   * Uses angle measurements from rising or falling edge methods to correct the odometer's theta
   * reading.
   */
  private void correctAngle() {
    double[] position = odometer.getXYT();
    double correctedTheta;
    if (alpha < beta) {
      correctedTheta = position[2] + 45 - ((alpha + beta) / 2);
    } else {
      correctedTheta = position[2] + 225 - ((alpha + beta) / 2);
    }
    odometer.setTheta(correctedTheta);
  }

}
