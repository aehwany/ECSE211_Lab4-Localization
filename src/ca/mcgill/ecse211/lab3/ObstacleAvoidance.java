package ca.mcgill.ecse211.lab3;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;

public class ObstacleAvoidance implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private float[] usData;
	private SampleProvider usDistance ;
	private final double TRACK;
	private final double WHEEL_RAD;
	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final double tileLength = 30.48;
	double currentT, currentY, currentX;
	double dx, dy, dt;
	double distanceToTravel;
	int iterator = 0;
	private Odometer odometer;
	private OdometerData odoData;
	int[][] path;
	/**
	 * creates an obstacle instance.
	 * @param leftMotor left motor object
	 * @param rightMotor right motor object
	 * @param TRACK track value
	 * @param WHEEL_RAD wheel radius
	 * @param finalPath map coordinates list
	 * 
	 * @throws Exception
	 */
		public ObstacleAvoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
				final double TRACK, final double WHEEL_RAD, int[][] finalPath) throws OdometerExceptions { // constructor
			this.odometer = Odometer.getOdometer();
			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;
			odoData = OdometerData.getOdometerData();
			odoData.setXYT(0 , 0 , 0);
			this.TRACK = TRACK;
			this.WHEEL_RAD = WHEEL_RAD;
			this.path = finalPath;
			SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
			usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
			// this instance
			this.usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
			// returned
		}

		/**
		 * method to start moving the car in navigation mode
		 * @return void
		 */
		public void run() {
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
				motor.stop();
				motor.setAcceleration(300);  // reduced the acceleration to make it smooth
			}
			// wait 5 seconds
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// there is nothing to be done here because it is not expected that
				// the odometer will be interrupted by another thread
			}
			// implemented this for loop so that navigation will work for any number of points
			while(iterator < path.length) { //iterate through all the points 
				travelTo(path[iterator][0]*tileLength, path[iterator][1]*tileLength);
				iterator++;
			}
		}


		/**
		 * causes  the  robot  to  travel  to  the  absolute  field  location  (x,  y),  
		 * specified  in tilepoints.This  method  should  continuously  callturnTo(double theta)
		 * and  then set  the motor speed to forward(straight). This will make sure that your heading is updated
		 * until you reach your exact goal. This method will poll the odometer for information.
		 * @param x x-coordinate
		 * @param y y-coordinate 
		 * @return void
		 */
		void travelTo(double x, double y) {
			currentX = odometer.getXYT()[0];// get the position on the board
			currentY = odometer.getXYT()[1];
			currentT = odometer.getXYT()[2];

			dx = x- currentX;
			dy = y - currentY;
			distanceToTravel = Math.sqrt(dx*dx+dy*dy);
			if(dy>=0) {
				dt=Math.atan(dx/dy);
			}
			else if(dy<=0&&dx>=0) {
				dt=Math.atan(dx/dy)+Math.PI;
			}
			else {
				dt=Math.atan(dx/dy)-Math.PI;
			}//Mathematical convention

			// initial angle is 0||2pi, same direction as y-axis, going clockwise
			double differenceInTheta = (dt*180/Math.PI-currentT); // robot has to turn "differenceInTheta",
			//turn the robot to the desired direction
			turnTo(differenceInTheta); 

			// drive forward required distance
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);

			while(isNavigating()) { //avoiding the obstacles
				usDistance.fetchSample(usData,0);
				float distance = usData[0]*100;
				if(distance<= 15) {	
					//if y is increasing, that's when the angle range is 270~360 or 0~90
					if ((odometer.getXYT()[2] > 270 && odometer.getXYT()[2] < 360) || (odometer.getXYT()[2] > 0 && odometer.getXYT()[2] < 90)) 
					{
						//turn left
						if(odometer.getXYT()[0]<2.4*30.48&&odometer.getXYT()[0]>1.3*30.48&&odometer.getXYT()[1]<2.5*30.48&&odometer.getXYT()[1]>1.6*30.48){
							leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);  // turn when facing obstacle and travel a certain distance and then turn again 
							rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);// then travel a certain distance
							leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
							rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
							leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
							rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
							leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
							rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
						}
						else {		//turn right
						leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);  // turn when facing obstacle and travel a certain distance and then turn again 
						rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);// then travel a certain distance
						leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
						rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
						leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
						rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
						leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
						rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
						}
					}
					else {	//if y is decreasing, that's when the angle range is 90~270 	
						//turn right
						if(odometer.getXYT()[0]<2.4*30.48&&odometer.getXYT()[0]>1.3*30.48&&odometer.getXYT()[1]<2.5*30.48&&odometer.getXYT()[1]>1.6*30.48){
							leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);  // turn when facing obstacle and travel a certain distance and then turn again 
							rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);// then travel a certain distance
							leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
							rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
							leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
							rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
							leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
							rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
						}
						else {		//turn left
							leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);  // turn when facing obstacle and travel a certain distance and then turn again 
							rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);// then travel a certain distance
							leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
							rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
							leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
							rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
							leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
							rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
						}
					}
					
					iterator--;
				}
			}
		}

		/**
		 *	This method causes the robot to turn (on point) to the absolute heading theta. 
		 *  This method should turn a MINIMAL angle to its target.
		 *  @param theta turn angle value
		 *  @return void
		 */
		void turnTo(double theta) {
			if(theta>180) { // angle convention. the robot should turn in direction
				theta=360-theta;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
				rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);
			}
			else if(theta<-180) {
				theta=360+theta;
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
				rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
			}
			else {
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
				rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);	
			}
		}

		/**
		 * This  method  returns  true  if  another  thread  has  called travelTo()
		 * or turnTo()and  the method has yet to return; false otherwise.    
		 * @return boolean 
		 */
		boolean isNavigating() {
			if((leftMotor.isMoving() || rightMotor.isMoving()))
				return true;
			else 
				return false;

		}

		/**
		 * This  method  converts target distance to wheel rotation.
		 * @param radius radius of wheel
		 * @param distance target distance
		 * @return the wheel rotation 
		 */
		private static int convertDistance(double radius, double distance) {
			return (int) ((180.0 * distance) / (Math.PI * radius));
		}
		/**
		 * This  method  converts target distance to wheel rotation.
		 * @param radius radius of wheel
		 * @param width track value
		 * @param angle target turn angle 
		 * @return the wheel rotation 
		 */
		private static int convertAngle(double radius, double width, double angle) {
			return convertDistance(radius, Math.PI * width * angle / 360.0);
		}
}
