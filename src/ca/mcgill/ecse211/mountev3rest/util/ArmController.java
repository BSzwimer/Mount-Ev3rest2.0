package ca.mcgill.ecse211.mountev3rest.util;

import ca.mcgill.ecse211.mountev3rest.navigation.Navigation;
import ca.mcgill.ecse211.mountev3rest.sensor.ColorDetector;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * Provides a set of methods to perform the basic subtasks required for ring collection.
 * 
 * Among the facilities provided by the {@code ArmController} class are ring identification, and
 * ring retrieval.
 * 
 * @author angelortiz
 *
 */
public class ArmController {

  // Constants
  private static final int DISTANCE_TO_TREE = 14;
  private static final int COLOR_DETECTION_PERIOD = 15;
  private static final int FORWARD_SPEED = 100;
  private static final int OPEN_ANGLE = 80;

  private EV3MediumRegulatedMotor colorSensorMotor;
  private EV3MediumRegulatedMotor armMotor;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private Navigation navigation;
  private ColorDetector colorDetector;

  /**
   * Creates an {@code ArmController} that will operate on the provided motors.
   * 
   * @param colorSensorMotor Motor that will be used to control the light sensor.
   * @param armMotor Motor that will be used to control movement of the arm's claw.
   * @param leftMotor Left motor used to displace the robot.
   * @param rightMotor Right motor used to displace the robot.
   * @param navigation Navigation object used to move the robot the robot to different points on the
   *        grid.
   * @param colorDetector Object used to classify the reading of the light sensor into colors.
   * 
   * @see ColorDetector
   */
  public ArmController(EV3MediumRegulatedMotor colorSensorMotor, EV3MediumRegulatedMotor armMotor,
      EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigation,
      ColorDetector colorDetector) {
    this.colorSensorMotor = colorSensorMotor;
    this.armMotor = armMotor;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.navigation = navigation;
    this.colorDetector = colorDetector;
  }

  /**
   * Gets a ring from the tree face that the robot is looking at when the method is called. This
   * method assumes that the robot is already properly aligned with the corresponding tree face. A
   * number of beeps is produced before the ring is grabbed corresponding to the color of the ring
   * as defined by the {@code ColorDetector}'s encoding.
   * 
   * @see ColorDetector
   */
  public void getRing() {

    // Open the claw
    armMotor.setSpeed(100);
    armMotor.rotate(OPEN_ANGLE, true);
    
    // Position the color sensor
    colorSensorMotor.setSpeed(70);
    colorSensorMotor.rotate(-45, false);

    // Approach the tree
    leftMotor.setSpeed((int)(FORWARD_SPEED * navigation.MOTOR_OFFSET));
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, DISTANCE_TO_TREE), true);
    rightMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, DISTANCE_TO_TREE), false);
    
    // Sweep the color sensor to detect the ring color
    colorSensorMotor.setSpeed(10);
    int colorDetected = 5; // 5 means no detection
    int startTacho = colorSensorMotor.getTachoCount();
    int endTacho = 0;
    
    colorSensorMotor.rotate(70, true);
    
    // Check for color until the motor finished the sweep
    while(colorSensorMotor.isMoving()) {
      // Keep checking for color if no one has been detected
      colorDetected = colorDetector.getColor();
      if (colorDetected != 5) {
        endTacho = colorSensorMotor.getTachoCount();
        colorSensorMotor.stop(true);
        break;
      }
        
      try {
        Thread.sleep(COLOR_DETECTION_PERIOD);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    
    // If a color is detected stop the sweeping and beep
    if (colorDetected != 5) {
      for (int j = 0; j < colorDetected; j++)
        Sound.beep();
    }  
    
    // Reverse the rotation of the motor location
    colorSensorMotor.setSpeed(70);
    colorSensorMotor.rotate(startTacho - endTacho, false);

    // Close the claw again
    armMotor.setSpeed(50);
    armMotor.rotate(-OPEN_ANGLE, false);
    
    // Return the sensor to the original location
    colorSensorMotor.rotate(45, false);

    // Go back to the original point
    leftMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, -DISTANCE_TO_TREE), true);
    rightMotor.rotate(Navigation.convertDistance(navigation.WHEEL_RADIUS, -DISTANCE_TO_TREE), false);
  }

  /**
   * Releases the ring being carried by the robot on the arm if any.
   */
  public void releaseRing() {

    // speed needs to be tested
    armMotor.setSpeed(75);
    armMotor.rotate(360, false);

  }

}
