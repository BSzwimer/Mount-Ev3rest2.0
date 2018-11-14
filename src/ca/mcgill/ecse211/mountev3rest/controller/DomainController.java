package ca.mcgill.ecse211.mountev3rest.controller;

import java.util.logging.Handler;
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
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
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
 * a call to {@code crossTunnel(3, 3, 4, 6)} will not produce the expected outcome if the odometer
 * of the robot has not been set to the right initial values through the {@code localize()} method.
 * 
 * @see MetaController
 * 
 * @author angelortiz
 *
 */
public class DomainController {

  // Constants
  private static final double TRACK = 8.80;
  private static final double WHEEL_RADIUS = 2.05;
  private static final double TILE_SIZE = 30.48;
  private static final double MOTOR_OFFSET = 1.02; // TEST BATTERY IS 8V FOR BEST VALUE.
  private static final double SENSOR_OFFSET = 12.5;
  private static final boolean TRAJECTORY_CORRECTION = true;

  // Attributes
  Odometer odometer;
  Navigation navigation;
  Localizer localizer;
  UltrasonicPoller usPoller;
  LightPoller lightPoller;
  ArmController armController;
  ColorDetector colorDetector;

  // Temporal
  EV3LargeRegulatedMotor leftMotor;
  EV3LargeRegulatedMotor rightMotor;

  // Threads
  Thread odoThread;
  Thread navThread;
  Thread lightThread;

  /**
   * Creates a {@code DomainController} and initializes all the required specialized classes.
   * <p>
   * Instantiation of the {@code DomainController} involves initializing the singleton
   * {@code Odometer}, {@code LightPoller}, and {@code UltrasonicPoller} classes, which is required
   * for the instantiation of other classes.
   * 
   * @throws OdometerException If there is a problem while creating the Odometer singleton instance.
   * @throws PollerException If the UltrasonicPoller or LightPoller have not been instantiated.
   * 
   * @see Odometer
   * @see LightPoller
   * @see UltrasonicPoller
   */
  public DomainController() throws OdometerException, PollerException {

    // Get motor objects
    leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    EV3MediumRegulatedMotor colorSensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
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
    navigation = new Navigation(leftMotor, rightMotor, TRAJECTORY_CORRECTION, WHEEL_RADIUS, TRACK,
        MOTOR_OFFSET, SENSOR_OFFSET);
    localizer = new Localizer(leftMotor, rightMotor, navigation, SENSOR_OFFSET, TILE_SIZE);
    colorDetector = new ColorDetector(LocalEV3.get().getTextLCD(), lightPoller);
    armController = new ArmController(colorSensorMotor, armMotor, leftMotor, rightMotor, navigation, colorDetector);

    // Initialize and start the required extra threads
    odoThread = new Thread(odometer);
    navThread = new Thread(navigation);
    lightThread = new Thread(lightPoller);
    odoThread.start();
    navThread.start();
    lightThread.start();
  }

  /**
   * Uses the {@code Localizer} class to provide the odometer with an initial set of coordinates
   * that correspond to the location of the robot with respect to the grid.
   * 
   * @see Localizer
   */
  public void localize(int startingCorner) {
    localizer.localize(0, 8, 8);
  }

  /**
   * Crosses the tunnel specified by the given coordinates. The method makes sure that the robot
   * reaches the closest entrance of the tunnel, then it moves through it until the robot is
   * completely outside on the other side.
   * 
   * @param BR_LL_x Lower left X coordinate of the tunnel.
   * @param BR_LL_y Lower left Y coordinate of the tunnel.
   * @param BR_UR_x Upper right X coordinate of the tunnel.
   * @param BR_UR_y Upper right Y coordinate of the tunnel.
   */
  public void crossTunnel(int BR_LL_x, int BR_LL_y, int BR_UR_x, int BR_UR_y) {
    double LL_dist = navigation.computeDistance(BR_LL_x, BR_LL_y);
    double UR_dist = navigation.computeDistance(BR_UR_x, BR_UR_y);

    boolean wasEnabled = navigation.isCorrectionEnabled();

    if (BR_UR_x - BR_LL_x == 2) { // Bridge is placed horizontally.
      if (LL_dist < UR_dist) { // Robot is closer to the lower left corner.
        navigation.travelTo(BR_LL_x - 1, BR_LL_y + 0.5);
        navigation.waitNavigation();
        navigation.travelTo(BR_LL_x - 0.5, BR_LL_y + 0.5);
        navigation.waitNavigation();
        navigation.disableCorrection();
        navigation.travelTo(BR_UR_x + 1, BR_UR_y - 0.5);
        navigation.waitNavigation();
      } else { // Robot is closer to the upper right corner.
        navigation.travelTo(BR_UR_x + 1, BR_UR_y - 0.5);
        navigation.waitNavigation();
        navigation.travelTo(BR_UR_x + 0.5, BR_UR_y - 0.5);
        navigation.waitNavigation();
        navigation.disableCorrection();
        navigation.travelTo(BR_LL_x - 1, BR_LL_y + 0.5);
        navigation.waitNavigation();
      }
    } else { // Bridge is placed vertically.
      if (LL_dist < UR_dist) { // Robot is closer to the lower left corner.
        navigation.travelTo(BR_LL_x + 0.5, BR_LL_y - 1);
        navigation.waitNavigation();
        navigation.disableCorrection();
        navigation.travelTo(BR_UR_x - 0.5, BR_UR_y + 1);
        navigation.waitNavigation();
      } else { // Robot is closer to the upper right corner.
        navigation.travelTo(BR_UR_x - 0.5, BR_UR_y + 1);
        navigation.waitNavigation();
        navigation.disableCorrection();
        navigation.travelTo(BR_LL_x + 0.5, BR_LL_y - 1);
        navigation.waitNavigation();
      }
    }
    if (wasEnabled)
      navigation.enableCorrection();
  }

  /**
   * Approaches the tree given by the coordinates and positions the robot looking straight into a
   * particular face of the tree. The faces of the tree are defined by the range [0, 3] counting
   * counter-clockwise and starting from the face looking South.
   * 
   * @param TG_x X coordinate of the tree.
   * @param TG_y Y coordinate of the tree.
   * @param face Face of the tree towards which the robot will be positioned.
   */
  public void approachTree(int TG_x, int TG_y, int face) {
    // TODO
  }

  /**
   * Approaches the closest face of the ring tree and attempts to grab a ring from the higher
   * location. The light sensor if then used to identify a potential ring color. If no ring is
   * identified, then the robot attempts to grab a ring from the lower position of the face. Again,
   * color detection is used to identify a potential ring. If no ring is detected the robot moves to
   * the next face in the counter-clockwise direction. This keeps going until the robot finds a ring
   * or visits the four faces of the tree.
   * <p>
   * The robot takes an argument denoting the location of the last ring found to accelerate the
   * search process. The encoding for the ring location is the range {@code [1-8]} where 1 and 2
   * represent the high and low locations of the tree face looking South respectively and the rest
   * are defined similarly with the numbers incrementing in the counter-clockwise direction. For
   * instance,
   * <p>
   * {@code lastRingFound = 4}
   * <p>
   * Represents the high location of the tree face looking North.
   * 
   * @param lastRingFound Location of the last ring found as defined by the encoding. Zero means
   *        that no rings have been found yet.
   * 
   * @return Location of the ring found.
   * 
   * @see ColorDetector
   * @see ArmController
   */
  public int grabRings(int lastRingFound) {
	boolean wasEnabled = navigation.isCorrectionEnabled();
	navigation.disableCorrection();
    armController.getRing();
    if (wasEnabled) {
      navigation.enableCorrection();
    }
    return 0;
  }

  /**
   * Uses the {@code Localizer} and {@code Navigation} classes to move the robot through a set of
   * defined points for the purpose of testing the navigation capabilities of the system.
   * Additionally, the method displays the X, Y and Theta parameters of the odometer on the display.
   * <p>
   * Localization is optional and can be skipped using the parameters for quick testing of
   * navigation only.
   * 
   * @param localize Tells the method whether it should localize before starting navigation.
   * @param startCorner Starting corner of the robot on the grid as defined by the {@code Localizer}
   *        class.
   * @param points Array of X and Y coordinates to traverse.
   * @param lcd Display to use to show the odometer values.
   * 
   * @throws OdometerException If the odometer has not been instantiated.
   * 
   * @see Localizer
   * @see Navigation
   */
  public void testNavigation(boolean localize, int startCorner, double[][] points, TextLCD lcd)
      throws OdometerException {
    Display display = new Display(lcd);
    Thread disThread = new Thread(display);
    disThread.start();

    localizer.localize(startCorner, 8, 8);
    // navigation.travelTo(0, 1);
    // navigation.waitNavigation();
    /*navigation.travelTo(4, 1);
    navigation.waitNavigation();
    
    Button.waitForAnyPress();
    
    navigation.travelTo(4, 5);
    navigation.waitNavigation();
    
    Button.waitForAnyPress();*/
    
    //navigation.disableCorrection();
    
    /*leftMotor.setSpeed(150);
    rightMotor.setSpeed(150);
    leftMotor.forward();
    rightMotor.forward();
    
    Button.waitForAnyPress();*/

    odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
    crossTunnel(2, 3, 4, 4);
    localizer.tunnelLocalization();
    navigation.travelTo(5, 5);
    navigation.waitNavigation();
    grabRings(0);

  }

  public void testColorDetection(TextLCD lcd) {
    ColorDetector cd = new ColorDetector(lcd, lightPoller);
    cd.demoDetection();
  }

}
