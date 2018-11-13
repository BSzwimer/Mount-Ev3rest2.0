package ca.mcgill.ecse211.mountev3rest.navigation;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.mountev3rest.navigation.Odometer;
import ca.mcgill.ecse211.mountev3rest.navigation.OdometerException;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to display the content of the odometer variables X, Y and Theta
 */
public class Display implements Runnable {

  private Odometer odo;
  private TextLCD lcd;
  private OdometryCorrector corrector;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * Created a display object.
   * 
   * @param lcd LCD display to use to display the odometer parameters.
   * 
   * @throws OdometerException If the odometed has not been instantiated.
   */
  public Display(TextLCD lcd) throws OdometerException {
    odo = Odometer.getOdometer();
    this.lcd = lcd;
  }
  
  //REMOVE
  public Display(TextLCD lcd, OdometryCorrector corrector) throws OdometerException {
    odo = Odometer.getOdometer();
    this.corrector = corrector;
    this.lcd = lcd;
  }

  /*
   * Main logic of the Display class where the odometer values are constantly polled and displayed.
   * 
   * (non-Javadoc)
   * @see java.lang.Runnable#run()
   */
  public void run() {
    lcd.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odo.getXYT();
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      
      if (corrector != null)
        lcd.drawString("Corrector: " + (corrector.isEnabled() ? "ON " : "OFF"), 0, 4);
      
      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

}
