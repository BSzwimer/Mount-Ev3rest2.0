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
  private static final int US_LOCALIZATION_PERIOD = 25;
  private static final int MIN_US_DETECTIONS = 1;
  private static final int ROTATE_SPEED = 200;
  private static final int FORWARD_SPEED = 100;
  private static final int APROX_DIST = 7;
  private final double SENSOR_OFFSET;
  private final double TILE_SIZE;

  // Object Attributes
  private Odometer odometer;
  private OdometryCorrector odometryCorrector;
  private UltrasonicPoller usPoller;
  private LightPoller lightPoller;
  private Navigation navigation;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  // Localization Attributes
  private int prevDistance;
  private int currDistance;
  private double alpha;
  private double beta;

  /**
   * Creates a {@code Localizator} that will operate on the singleton {@code Odometer} class.
   * 
   * @param leftMotor Lower left motor of the robot.
   * @param rightMotor Lower right motor of the robot.
   * @param navigation Navigation object to move the robot on the grid.
   * @param odometryCorrector Object used to align the robot to a line during light localization.
   * @param SENSOR_OFFSET Distance between the lower light sensor and the robot's center.
   * @param TILE_SIZE Size of a tile in the grid in centimeters.
   * 
   * @throws OdometerException If the {@code Odometer} has not been instantiated.
   * @throws PollerException If the {@code UltrasonicPoller} of the {@code LightPoller} classes have
   *         not been instantiated.
   * 
   * @see Odometer
   */
  public Localizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      Navigation navigation, OdometryCorrector odometryCorrector, final double SENSOR_OFFSET,
      final double TILE_SIZE) throws OdometerException, PollerException {
    usPoller = UltrasonicPoller.getUltrasonicPoller();
    lightPoller = LightPoller.getLightPoller();
    odometer = Odometer.getOdometer();
    this.odometryCorrector = odometryCorrector;

    this.navigation = navigation;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.SENSOR_OFFSET = SENSOR_OFFSET;
    this.TILE_SIZE = TILE_SIZE;
  }

  /**
   * Performs ultrasonic localization to correct the angle Theta of the odometer and then it uses
   * light localization to get the correct values of the X and Y coordinates of the robot's
   * location. The values obtained are used to overwrite the odometer's current values.
   * <p>
   * The starting corner parameter is used to adjust the odometer values according the the absolute
   * set of coordinated on the map. The corners are defined using the range {@code [0-3]} with 0
   * being the lower left corner and the range increasing in the counter-clockwise direction.
   * 
   * @param startingCorner Starting corner of the robot on the grid.
   * @param LL_x X coordinate of the lower left corner of the robot's team area.
   * @param LL_y Y coordinate of the lower left corner of the robot's team area.
   * @param UR_x X coordinate of the upper right corner of the robot's team area.
   * @param UR_y Y coordinate of the upper right corner of the robot's team area.
   */
  public void localize(long startingCorner, long LL_x, long LL_y, long UR_x, long UR_y) {
    ultrasonicLocalization(startingCorner);
    lightLocalization(startingCorner, LL_x, LL_y, UR_x, UR_y);
  }

  /**
   * Estimates the angle of the robot with respect to the grid by rotating until a rising edge is
   * detected with the wall on each side. The angle in the middle of the two rising edge detections
   * is used to estimate the robot's real angle.
   * 
   * @param startingCorner Starting corner of the robot on the grid.
   */
  private void ultrasonicLocalization(long startingCorner) {
    boolean wasEnabled = odometryCorrector.isEnabled();
    odometryCorrector.disable();

    long updateStart, updateEnd;
    prevDistance = -1;
    boolean firstSearch = true;
    int counter = 0;

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();

    boolean highDelta = false;

    // Localization
    while (true) {
      updateStart = System.currentTimeMillis();
      currDistance = usPoller.poll();
      if (prevDistance < 0) {
        prevDistance = currDistance;
        continue;
      }

      /*System.out.println(String.format("Prev: %d, Curr: %d  |  Count: %d",
          prevDistance, currDistance, counter));*/

      // FALLING EDGE
      if (prevDistance > currDistance) { // Distance decreasing
        counter++;
        if (prevDistance - currDistance > 70)
          highDelta = true;
        else
          highDelta = false;
      } else if (prevDistance < currDistance) {
        counter = 0;
        highDelta = false;
        //System.out.println("OFF");
      }
      
      // Check if edge conditions are met
      if /*(counter >= MIN_US_DETECTIONS && highDelta && currDistance < 35)*/ (currDistance < 25 && prevDistance > 25) {
        if (firstSearch) {
          beta = odometer.getXYT()[2];
          leftMotor.backward();
          rightMotor.forward();

          firstSearch = false;
          counter = 0;
          highDelta = false;
          
          //System.out.println("FIRST");

          // Don't start checking again right away
          try {
            Thread.sleep(500);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        } else {
          alpha = odometer.getXYT()[2];
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);

          //System.out.println("SECOND");
          
          break;
        }
      }
      
      prevDistance = currDistance;
      // This ensures that the localizer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < US_LOCALIZATION_PERIOD) {
        try {
          Thread.sleep(US_LOCALIZATION_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
        }
      }
    }
    correctAngle();

    // Adjust to starting position
    switch ((int) startingCorner) {
      case 0:
        break;
      case 1:
        odometer.update(0, 0, -90);
        break;
      case 2:
        odometer.update(0, 0, 180);
        break;
      case 3:
        odometer.update(0, 0, 90);
        break;
    }

    if (wasEnabled)
      odometryCorrector.enable();
  }

  /**
   * Provides a more accurate set of values for X, Y and Theta to the odometer by using light
   * localization.
   * <p>
   * The light localization routine consists of moving forward in the X and Y directions until a
   * line is detected. Once the detection occurs, the robot is aligned with the line under it using
   * the other light sensor. Then both the Theta and X or Y coordinates are updated on the odometer
   * using the current knowledge of the robots location.
   * <p>
   * The starting corner parameter allows the method to the initial coordinates of the robot
   * according to the absolute coordinate system defined by the grid.
   * 
   * @param startingCorner Starting corner of the robot on the grid.
   * @param LL_x X coordinate of the lower left corner of the robot's team area.
   * @param LL_y Y coordinate of the lower left corner of the robot's team area.
   * @param UR_x X coordinate of the upper right corner of the robot's team area.
   * @param UR_y Y coordinate of the upper right corner of the robot's team area.
   */
  public void lightLocalization(long startingCorner, long LL_x, long LL_y, long UR_x, long UR_y) {
    boolean wasEnabled = odometryCorrector.isEnabled();
    odometryCorrector.disable();

    // Try to get closer to the nearest intersection
    switch ((int) startingCorner) {
      case 0:
        navigation.turnTo(90);
        break;
      case 1:
        navigation.turnTo(0);
        break;
      case 2:
        navigation.turnTo(270);
        break;
      case 3:
        navigation.turnTo(180);
        break;
    }
    
    /*leftMotor.setSpeed((int)(FORWARD_SPEED * navigation.MOTOR_OFFSET));
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
    
    lightPoller.poll();
    while(!lightPoller.leftInLine && !lightPoller.rightInLine) {
      try {
        Thread.sleep(LOCALIZATION_PERIOD);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      lightPoller.poll();
    }*/
    
    navigation.advanceDist(APROX_DIST);
    navigation.waitNavigation();

    // Localize in Y
    // Rotate to look into the +Y direction
    switch ((int) startingCorner) {
      case 0:
        navigation.turnTo(0);
        break;
      case 1:
        navigation.turnTo(0);
        break;
      case 2:
        navigation.turnTo(180);
        break;
      case 3:
        navigation.turnTo(180);
        break;
    }

    // Move forward until a line is detected and the robot is aligned with it
    findLine();

    // Now correct the values of the odometer.
    double[] currPosition = odometer.getXYT();
    switch ((int) startingCorner) {
      case 0:
        odometer.setXYT(currPosition[0], SENSOR_OFFSET, 0);
        break;
      case 1:
        odometer.setXYT(currPosition[0], SENSOR_OFFSET, 0);
        break;
      case 2:
        odometer.setXYT(currPosition[0], -SENSOR_OFFSET, 180);
        break;
      case 3:
        odometer.setXYT(currPosition[0], -SENSOR_OFFSET, 180);
        break;
    }

    // Position the axis of the robot back on the line
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, -SENSOR_OFFSET), true);
    rightMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, -SENSOR_OFFSET), false);

    // Localize in X
    // Rotate to look into the +X direction
    switch ((int) startingCorner) {
      case 0:
        navigation.turnTo(90);
        break;
      case 1:
        navigation.turnTo(270);
        break;
      case 2:
        navigation.turnTo(270);
        break;
      case 3:
        navigation.turnTo(90);
        break;
    }

    // Move forward until a line is detected and the robot is aligned with it
    findLine();

    // Now correct the values of the odometer.
    currPosition = odometer.getXYT();
    switch ((int) startingCorner) {
      case 0:
        odometer.setXYT(SENSOR_OFFSET, currPosition[1], 90);
        break;
      case 1:
        odometer.setXYT(-SENSOR_OFFSET, currPosition[1], 270);
        break;
      case 2:
        odometer.setXYT(-SENSOR_OFFSET, currPosition[1], 270);
        break;
      case 3:
        odometer.setXYT(SENSOR_OFFSET, currPosition[1], 90);
        break;
    }

    // Position the axis of the robot back on the line
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, -SENSOR_OFFSET), true);
    rightMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, -SENSOR_OFFSET), false);


    // Adjust the correction for the starting corner of the robot
    switch ((int) startingCorner) {
      case 0:
        odometer.update(TILE_SIZE * (LL_x + 1), TILE_SIZE * (LL_y + 1), 0);
        break;
      case 1:
        odometer.update(TILE_SIZE * (UR_x - 1), TILE_SIZE * (LL_y + 1), 0);
        break;
      case 2:
        odometer.update(TILE_SIZE * (UR_x - 1), TILE_SIZE * (UR_y - 1), 0);
        break;
      case 3:
        odometer.update(TILE_SIZE * (LL_x + 1), TILE_SIZE * (UR_y - 1), 0);
        break;
    }

    if (wasEnabled)
      odometryCorrector.enable();
  }

  /**
   * Moves forward until one of the sensors detects a line. Once a line is detected a
   * {@code OdometryCorrector} method is called to align the other side.
   */
  private void findLine() {
    leftMotor.setSpeed((int) (FORWARD_SPEED * navigation.MOTOR_OFFSET));
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.forward();
    rightMotor.forward();

    lightPoller.poll();
    while (!lightPoller.leftInLine && !lightPoller.rightInLine) {
      try {
        Thread.sleep(LOCALIZATION_PERIOD);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      lightPoller.poll();
    }

    boolean lineDetected = false;

    lightPoller.poll();
    while (!lineDetected) {
      if (lightPoller.leftInLine && lightPoller.rightInLine) {
        return;
      } else if (lightPoller.leftInLine)
        lineDetected = odometryCorrector.rotateUntilDetection(1);
      else if (lightPoller.rightInLine)
        lineDetected = odometryCorrector.rotateUntilDetection(0);

      lightPoller.poll();
    }
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

    correctedTheta = (correctedTheta + 180) % 360;

    odometer.setTheta(correctedTheta);
  }

}
