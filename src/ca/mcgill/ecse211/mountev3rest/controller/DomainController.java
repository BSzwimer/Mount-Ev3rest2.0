package ca.mcgill.ecse211.mountev3rest.controller;

import ca.mcgill.ecse211.mountev3rest.navigation.Display;
import ca.mcgill.ecse211.mountev3rest.navigation.Localizer;
import ca.mcgill.ecse211.mountev3rest.navigation.Navigation;
import ca.mcgill.ecse211.mountev3rest.navigation.Odometer;
import ca.mcgill.ecse211.mountev3rest.navigation.OdometerException;
import ca.mcgill.ecse211.mountev3rest.navigation.OdometryCorrector;
import ca.mcgill.ecse211.mountev3rest.sensor.ColorDetector;
import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;
import ca.mcgill.ecse211.mountev3rest.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.mountev3rest.util.ArmController;
import ca.mcgill.ecse211.mountev3rest.util.CoordinateMap;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Provides an interface to execute the main subtasks required for the robot to perform the overall
 * routine.
 * <p>
 * The {@code DomainController} represents the low layer of the controller in the system. The class
 * is not concerned with the general state of the main routine execution. Instead, the
 * {@code DomainController} handles all the details related to each of the subtasks required for the
 * main routine. This means that internally, the {@code DomainCotroller} handles all the required
 * calls to more specialized classes and ensures that the requested subtask is completed.
 * <p>
 * It is assumed that the caller of the {@code DomainController} methods (usually the
 * {@code MetaController}) will conform to the external requirements of each subtask. For instance,
 * a call to {@code crossTunnel()} will not produce the expected outcome if the odometer of the
 * robot has not been set to the right initial values through the {@code localize()} method.
 * 
 * @see MetaController
 * 
 * @author angelortiz
 *
 */
public class DomainController {

  // Constants
  private static final double TRACK = 8.50;
  private static final double WHEEL_RADIUS = 2.05;
  private static final double TILE_SIZE = 30.48;
  private static final double MOTOR_OFFSET = 1.02;
  private static final double SENSOR_OFFSET = -2.3;

  // Attributes
  CoordinateMap map;
  Odometer odometer;
  OdometryCorrector odometryCorrector;
  Navigation navigation;
  Localizer localizer;
  UltrasonicPoller usPoller;
  LightPoller lightPoller;
  ArmController armController;
  ColorDetector colorDetector;

  // Threads
  Thread odoThread;
  Thread navThread;

  /**
   * Creates a {@code DomainController} and initializes all the required specialized classes.
   * <p>
   * Instantiation of the {@code DomainController} involves initializing the singleton
   * {@code Odometer}, {@code LightPoller}, and {@code UltrasonicPoller} classes, which is required
   * for the instantiation of other classes.
   * 
   * @throws OdometerException If there is a problem while creating the Odometer singleton instance.
   * @throws PollerException If there is a problem while instantiating the UltrasonicPoller an
   *         LightPoller objects.
   * 
   * @see Odometer
   * @see LightPoller
   * @see UltrasonicPoller
   */
  public DomainController() throws OdometerException, PollerException {

    // Get motor objects
    EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    EV3MediumRegulatedMotor colorSensorMotor =
        new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
    EV3MediumRegulatedMotor armMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

    // Instantiate the sensors
    EV3ColorSensor rightLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    EV3ColorSensor topLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
    EV3ColorSensor leftLightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
    EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));

    // Create the specialized objects
    usPoller = UltrasonicPoller.getUltrasonicPoller(usSensor);
    lightPoller = LightPoller.getLightPoller(topLightSensor, leftLightSensor, rightLightSensor);
    odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RADIUS, MOTOR_OFFSET);
    odometryCorrector =
        new OdometryCorrector(leftMotor, rightMotor, TILE_SIZE, SENSOR_OFFSET, MOTOR_OFFSET);
    navigation =
        new Navigation(leftMotor, rightMotor, odometryCorrector, WHEEL_RADIUS, TRACK, MOTOR_OFFSET);
    localizer = new Localizer(leftMotor, rightMotor, navigation, odometryCorrector, SENSOR_OFFSET,
        TILE_SIZE);
    colorDetector = new ColorDetector(LocalEV3.get().getTextLCD());
    armController = new ArmController(colorSensorMotor, armMotor, leftMotor, rightMotor, navigation,
        colorDetector);

    // Initialize and start the required extra threads
    odoThread = new Thread(odometer);
    navThread = new Thread(navigation);
    odoThread.start();
    navThread.start();


    // REMOVE
    odometer.setXYT(TILE_SIZE * 1, TILE_SIZE * 1, 0);
  }

  /**
   * Sets the map coordinates to be used during each of the tasks.
   * 
   * @param map Map to be used during the tasks.
   */
  public void setMap(CoordinateMap map) {
    this.map = map;
  }

  /**
   * Uses the {@code Localizer} class to provide the odometer with an initial set of coordinates
   * that correspond to the location of the robot with respect to the grid.
   * 
   * @see Localizer
   */
  public void localize() {
    localizer.localize(map.StartCorner, map.LL_x, map.LL_y, map.UR_x, map.UR_y);
  }

  /**
   * Crosses the tunnel specified by the map given coordinates. The method makes sure that the robot
   * reaches the closest entrance of the tunnel, then it moves through it until the robot is
   * completely outside on the other side.
   * 
   * @see CoordinateMap
   */
  public void crossTunnel() {
    double LL_dist = navigation.computeDistance(map.TN_LL_x, map.TN_LL_y);
    double UR_dist = navigation.computeDistance(map.TN_UR_x, map.TN_UR_y);

    boolean wasEnabled = odometryCorrector.isEnabled();

    if (map.TN_UR_x - map.TN_LL_x == 2) { // Bridge is placed horizontally.
      if (LL_dist < UR_dist) { // Robot is closer to the lower left corner.
        navigation.travelToX(map.TN_LL_x - 1);
        navigation.waitNavigation();
        Button.waitForAnyPress();
        navigation.travelToY(map.TN_LL_y + 0.5);
        navigation.waitNavigation();
        Button.waitForAnyPress();
        navigation.travelToX(map.TN_LL_x - 0.3);
        navigation.waitNavigation();
        Button.waitForAnyPress();
        odometryCorrector.disable();
        navigation.travelToX(map.TN_UR_x + 1);
        navigation.waitNavigation();
      } else { // Robot is closer to the upper right corner.
        navigation.travelToX(map.TN_UR_x + 1);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_UR_y - 0.5);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_UR_x + 0.3);
        navigation.waitNavigation();
        odometryCorrector.disable();
        navigation.travelToX(map.TN_LL_x - 1);
        navigation.waitNavigation();
      }
    } else { // Bridge is placed vertically.
      if (LL_dist < UR_dist) { // Robot is closer to the lower left corner.
        navigation.travelToY(map.TN_LL_y - 2);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_LL_x + 0.5);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_LL_y - 0.5);
        navigation.waitNavigation();
        odometryCorrector.disable();
        navigation.travelToY(map.TN_UR_y + 1);
        navigation.waitNavigation();
      } else { // Robot is closer to the upper right corner.
        navigation.travelToY(map.TN_UR_y + 1);
        navigation.waitNavigation();
        navigation.travelToX(map.TN_UR_x - 0.5);
        navigation.waitNavigation();
        navigation.travelToY(map.TN_UR_y + 0.3);
        navigation.waitNavigation();
        odometryCorrector.disable();
        navigation.travelToY(map.TN_LL_y - 1);
        navigation.waitNavigation();
      }
    }

    // localizer.tunnelLocalization();

    if (wasEnabled)
      odometryCorrector.enable();
  }

  /**
   * Approaches the robot to the tree containing the ring set and positions it looking into the
   * nearest face.
   */
  public void approachTree() {
    navigation.travelToY(map.T_y);
    navigation.waitNavigation();

    double[] position = odometer.getXYT();

    if (Math.abs(position[0] - TILE_SIZE * (map.T_x - 1)) < Math
        .abs(position[1] - TILE_SIZE * (map.T_x + 1))) {
      navigation.travelToX(map.T_x - 1);
    } else {
      navigation.travelToX(map.T_x + 1);
    }
    navigation.waitNavigation();
  }

  /**
   * Moves the robot to the next tree face in the counter-clockwise direction. This method assumes
   * the the robot is already looking into one of the three faces. This can be achieved by calling
   * {@code approachTree()}.
   */
  public void goToNextFace() {
    odometryCorrector.updateDirection();

    switch (odometryCorrector.direction) {
      case NORTH:
        navigation.travelToX(map.T_x + 1);
        navigation.waitNavigation();

        navigation.travelToY(map.T_y);
        navigation.waitNavigation();

        navigation.turnTo(270);
        break;
      case EAST:
        navigation.travelToY(map.T_y - 1);
        navigation.waitNavigation();

        navigation.travelToX(map.T_x);
        navigation.waitNavigation();

        navigation.turnTo(0);
        break;
      case SOUTH:
        navigation.travelToX(map.T_x - 1);
        navigation.waitNavigation();

        navigation.travelToY(map.T_y);
        navigation.waitNavigation();

        navigation.turnTo(90);
        break;
      case WEST:
        navigation.travelToY(map.T_y + 1);
        navigation.waitNavigation();

        navigation.travelToX(map.T_x);
        navigation.waitNavigation();

        navigation.turnTo(180);
        break;
      default:
        break;
    }
  }

  /**
   * Approaches the tree slowly, detects the color of a ring if any and attempts to get it. This
   * method assumes that the robot is already looking into a face of the tree and positioned on the
   * intersection right in front of it. If a color is detected a sequence of beeps matching the
   * color encoding of the {@code ColorDetector} is issued. Once the collection is completed the
   * robot goes back to the initial intersection where is was located before the method call.
   * 
   * @see ColorDetector
   * @see ArmController
   */
  public void grabRings() {
    boolean wasEnabled = odometryCorrector.isEnabled();

    odometryCorrector.disable();
    armController.getRing();

    if (wasEnabled)
      odometryCorrector.enable();
  }

  // REMOVE
  public void testNavigation() throws OdometerException {

    TextLCD lcd = LocalEV3.get().getTextLCD();
    lcd.drawString("      READY      ", 0, 4);

    // Button.waitForAnyPress();

    Display display = new Display(lcd);
    Thread disThread = new Thread(display);
    disThread.start();

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    //odometryCorrector.disable();

    odometer.setXYT(TILE_SIZE * 1, TILE_SIZE * 1, 0);
    navigation.travelToY(5);
    navigation.waitNavigation();
    navigation.travelToX(6);
    navigation.waitNavigation();
    
    /*navigation.turnTo(90);
    Button.waitForAnyPress();
    navigation.turnTo(180);
    Button.waitForAnyPress();
    navigation.turnTo(90);
    Button.waitForAnyPress();
    navigation.turnTo(0);
    Button.waitForAnyPress();*/

    lcd.clear();
    lcd.drawString("       DONE       ", 0, 4);

    Button.waitForAnyPress();
  }

  // REMOVE
  public void testColorDetection() {
    TextLCD lcd = LocalEV3.get().getTextLCD();
    ColorDetector cd = null;
    try {
      cd = new ColorDetector(lcd);
    } catch (PollerException e) {
      e.printStackTrace();
    }
    //cd.demoDetection();
    cd.printRed();
  }

}
