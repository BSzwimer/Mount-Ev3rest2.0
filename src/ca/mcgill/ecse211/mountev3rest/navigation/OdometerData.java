package ca.mcgill.ecse211.mountev3rest.navigation;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Singleton class representation of the odometer's data.
 * <p>
 * The {@code OdometerData} class holds the relevant information regarding the location of the
 * robot. Additionally, it provides a thread safe interface to get and set this values.
 * 
 * @see Odometer
 * @author angelortiz
 *
 */
public class OdometerData {

  // Attributes
  // Singleton instance
  private static OdometerData odoData = null;

  // Position parameters
  private volatile double x; // x-axis position
  private volatile double y; // y-axis position
  private volatile double theta; // Head angle

  // Class control variables
  private volatile static int numberOfIntances = 0; // Number of OdometerData
                                                    // objects instantiated
                                                    // so far
  private static final int MAX_INSTANCES = 1; // Maximum number of
                                              // OdometerData instances

  // Thread control flags
  private static Lock lock = new ReentrantLock(true); // Fair lock for
                                                      // concurrent writing
  private volatile boolean isReseting = false; // Indicates if a thread is
                                               // trying to reset any
                                               // position parameters
  private Condition doneReseting = lock.newCondition(); // Let other threads
                                                        // know that a reset
                                                        // operation is
                                                        // over.


  /**
   * Private default constructor. A factory is used instead to preserve the singleton class
   * condition.
   */
  protected OdometerData() {
    this.x = 0;
    this.y = 0;
    this.theta = 0;
  }

  /**
   * OdometerData factory method. Returns an OdometerData instance and makes sure that only one
   * instance is ever created. If the user tries to instantiate multiple objects, the method throws
   * a MultipleOdometerDataException.
   * 
   * @return An {@code OdometerData} object.
   * @throws OdometerException If instantiated more than once.
   */
  public synchronized static OdometerData getOdometerData() throws OdometerException {
    if (odoData != null) { // Return existing object
      return odoData;
    } else if (numberOfIntances < MAX_INSTANCES) { // create object and
                                                   // return it
      odoData = new OdometerData();
      numberOfIntances += 1;
      return odoData;
    } else {
      throw new OdometerException("Only one intance of the Odometer can be created.");
    }

  }

  /**
   * Returns the Odometer data, while making sure that no other object is manipulating the data at
   * the time.
   * 
   * @return The X, Y and Theta odometer values.
   */
  public double[] getXYT() {
    double[] position = new double[3];
    lock.lock();
    try {
      while (isReseting) { // If a reset operation is being executed, wait
        // until it is over.
        doneReseting.await(); // Using await() is lighter on the CPU
        // than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;

    } catch (InterruptedException e) {
      // Print exception to screen
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;

  }

  /**
   * Updates the odometer's X, Y and Theta values by adding the provided deltas corresponding to
   * each variables.
   * 
   * @param dx Delta value to add to X.
   * @param dy Delta value to add to X.
   * @param dtheta Delta value to add to Theta.
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isReseting = true;
    try {
      x += dx;
      y += dy;
      theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates
                                                    // within 360
                                                    // degrees
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }

  }

  /**
   * Overwrites the existing values of the odometer with those provided in the parameters.
   * 
   * @param x New value for X.
   * @param y New value for Y.
   * @param theta New value for Theta.
   */
  public void setXYT(double x, double y, double theta) {
    lock.lock();
    isReseting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites the value of X.
   * 
   * @param x New value of X.
   */
  public void setX(double x) {
    lock.lock();
    isReseting = true;
    try {
      this.x = x;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites the value of Y.
   * 
   * @param y New value of Y.
   */
  public void setY(double y) {
    lock.lock();
    isReseting = true;
    try {
      this.y = y;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites the value of Theta.
   * 
   * @param theta New value of theta.
   */
  public void setTheta(double theta) {
    lock.lock();
    isReseting = true;
    try {
      this.theta = theta;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

}
