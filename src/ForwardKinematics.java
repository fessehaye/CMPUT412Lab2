
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;

/*
 * Forward Kinematic java class used to solve question 4 & 5
 */
public class ForwardKinematics {
	//x & y motors
	static RegulatedMotor motorFar = new EV3LargeRegulatedMotor(MotorPort.B);
	static RegulatedMotor motorClose = new NXTRegulatedMotor(MotorPort.A);

	//length of each arm by millimeters
	static double armLenC = (8*29);//in mm
	static double armLenF = (8*14);// in mm

	public static void main(String[] args) {
		//for question 4
		getPosition(0,-90);
		//for question 5
		distanceMeasure();

	}
	/*
	 * Solution to exercise 4
	 * 
	 * Write a program that given two angles, the robot moves to their corresponding joint angles, and
	 * returns the (x,y) position of the end effector and Measure the accuracy/repeatability
	 * of your 2DOF robot.
	 * 
	 * uses TwoDOFAnalyticalsolution class
	 */
	public static void getPosition(double degC, double degF ){

		TwoDOFAnalyticalSolution first = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		motorFar.setSpeed(40);

		//rotates motors using angles given
		//remember when moving to add *7 to theta1 because of gear ratio
		motorClose.rotateTo((int)degC*7);
		motorFar.rotateTo((int)degF);

		//Calculates Angles with the tachometer of each motors
		first.setThetaC(motorClose.getTachoCount()/7);
		first.setThetaF(motorFar.getTachoCount());

		//Calculates x and y absolute position
		System.out.println("x: " + (int)(first.getXe()-(armLenC+armLenF)) + "mm");
		System.out.println("y: " + (int)first.getYe() + "mm");

		Button.waitForAnyPress();

		motorFar.rotateTo(0);
		motorClose.rotateTo(0);
	}
	/*
	 * Solution to exercise 5
	 * 
	 * Give your robot the functionality of measuring distances and angles in a 2D plane.
	 * 
	 * uses TwoDOFAnalyticalsolution class
	 */
	public static void distanceMeasure(){

		TwoDOFAnalyticalSolution first = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		TwoDOFAnalyticalSolution second = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		TwoDOFAnalyticalSolution third = new TwoDOFAnalyticalSolution(armLenC,armLenF);
		double d1, d2, d3, a1, a2, a3, xd, yd;
		//Records first point of the triangle
		System.out.println("Press to Record 1");
		Button.waitForAnyPress();

		//Calculates angles for the motor for the first point
		first.setThetaC(motorClose.getTachoCount()/7);
		first.setThetaF(motorFar.getTachoCount());

		//Records second point of the triangle
		System.out.println("Press to Record 2");
		Button.waitForAnyPress();

		//Calculates angles for the motor for the second point
		second.setThetaC(motorClose.getTachoCount()/7);
		second.setThetaF(motorFar.getTachoCount());

		//Records third point of the triangle
		System.out.println("Press to Record 3");
		Button.waitForAnyPress();

		//Calculates angles for the motor for the third point
		third.setThetaC(motorClose.getTachoCount()/7);
		third.setThetaF(motorFar.getTachoCount());

		//Calculates Distances between different points
		yd = second.getYe()-first.getYe();
		xd = second.getXe()-first.getXe();
		d1 = Math.sqrt((xd*xd)+(yd*yd));

		yd = third.getYe()-second.getYe();
		xd = third.getXe()-second.getXe();
		d2 = Math.sqrt((xd*xd)+(yd*yd));

		yd = first.getYe()-third.getYe();
		xd = first.getXe()-third.getXe();
		d3 = Math.sqrt((xd*xd)+(yd*yd));

		//Calculates angles within the triangle
		a3 = Math.abs(Math.acos(((d1*d1)-(d2*d2)-(d3*d3))/(-2*d2*d3))*180/Math.PI);
		a2 = Math.abs(Math.acos(((d3*d3)-(d2*d2)-(d1*d1))/(-2*d2*d1))*180/Math.PI);
		a1 = 180-a3-a2;

		System.out.println("1to2 " + (int)d1 + " " + (int)a1);
		System.out.println("2to3 " + (int)d2 + " " + (int)a2);
		System.out.println("3to1 " + (int)d3 + " " + (int)a3);

		Button.waitForAnyPress();

	}




}
