import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;

/*
 * Path planning java class used to solve question 8 & 9
 */
public class pathplanning {
	//length of each arm by millimeters
	static double armLenC = (8*29);//in mm
	static double armLenF = (8*14);// in mm

	// used for breaking up line into individual smaller ones
	static double increment = 5;// in mm

	//x & y motors
	static RegulatedMotor motorFar = new EV3LargeRegulatedMotor(MotorPort.B);
	static RegulatedMotor motorClose = new NXTRegulatedMotor(MotorPort.A);

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		//for part 1 of question 8
		line(0,-50,-120,-50);

		//for part 2 of question 8
		line2(0,0, 135, 100);

		//for part 3 of question 8
		double[][] points = {
				{0,0},
				{100,-50},
				{100,-75}
		};
		arc(points);

		//for question 9
		labyrinth();
	}

	/*
	 * Solution to exercise 9
	 * 
	 * Using the pencil attached to your robots end effector,
	 *  complete the labyrinth, draw the right path to the exit.
	 *  
	 *  uses arc and line function from previous exercise 
	 */
	private static void labyrinth() {
		//set motor speeds
		motorFar.setSpeed(40);
		//list coordinates relative to starting position into one array
		double[][] points = {
				{0,0},
				{40,-200},
				{30,-125},
				{-5, -135},
				{-42,-90},
				{-10,-60},
				{-60,-65},
				{-55,-105},
				{-80,-115},
				{-55, -65},
				{-100, -65},
				{-100, -115},
				{-115, -115},
				{-100, -65},
				{-140, -65},
				{-150, -105},
				{-140, -135},
				{-175, -160},
				{-175, -210},
		};
		//parse array into arc function
		arc(points);

		Button.waitForAnyPress();
		//reset position after done
		motorFar.rotateTo(0);
		motorClose.rotateTo(0);

	}
	/*
	 * Solution to exercise 8 part 3
	 * 
	 * Draw an arc defined by n points.
	 * 
	 * using an array of coordinates we break them up into smaller lines
	 * and run the line function on each pairing
	 * 
	 * uses line function from previous exercise 
	 */
	private static void arc(double[][] points) {
		// TODO Auto-generated method stub
		for(int i = 0; i<points.length - 1; i++){
			//run line function from previous exercise
			line(points[i][0],points[i][1],points[i+1][0],points[i+1][1]);
		}
	}

	/*
	 * Solution to exercise 8 part 1
	 * 
	 * Draw a straight line defined by two points.
	 * 
	 * 
	 */
	public static void line(double yS, double xS, double yF, double xF){
		motorFar.setSpeed(40);
		//starting position
		double[] cS = {yS,xS};
		//ending position
		double[] cF = {yF,xF};
		//increments in x
		double xi = 0;
		//increments in y
		double yi = 0;
		//angles for motors
		int[] thetas;

		//fines how to break up the line into many smaller lines
		double[] inc = findNumPoints(cS[0],cS[1],cF[0],cF[1]);

		//finds angles for the starting position and moves motors into position
		thetas = getThetas4Position(cS[0], cS[1]);
		motorClose.rotateTo(thetas[0],true);
		motorFar.rotateTo(thetas[1]); 

		while(motorClose.isMoving());

		//breaks up the line and goes to each individual points and moves to 
		// each point
		for(int i=0; i<inc[0]; i++){
			yi += inc[1];
			xi += inc[2];
			//finds angles to the next checkpoint
			thetas = getThetas4Position(cS[0]+yi, cS[1]+xi);
			//move to the next checkpoint
			motorClose.rotateTo(thetas[0],true);
			motorFar.rotateTo(thetas[1]); 

			while(motorClose.isMoving());
		}
		//moves to final position
		thetas = getThetas4Position(cF[0], cF[1]);
		motorClose.rotateTo(thetas[0],true);
		motorFar.rotateTo(thetas[1]); 

		while(motorClose.isMoving()); 
	}


	/*
	 * Helper function for line function to get the theta values to move
	 * to a specific point. 
	 */

	public static int[] getThetas4Position(double y, double x) {
		double d, theta1, theta2;
		//add arm length to x
		x += (armLenC+armLenF);
		//calculate distance
		d = ((x*x)+(y*y)-(armLenC*armLenC)-(armLenF*armLenF))/(2*armLenC*armLenF);

		theta2 = Math.atan2(Math.sqrt(1-(d*d)), d);
		theta1 = Math.atan2(y, x) - Math.atan2(armLenF*Math.sin(theta2),(armLenC+(armLenF*Math.cos(theta2))));

		theta1 = theta1*180/Math.PI;
		theta2 = theta2*180/Math.PI;

		//remember when moving to add *7 to theta1 because of gear ratio
		int[] solution = {(int)theta1*7,(int)theta2};

		return solution;
	}

	/*
	 * Helper function for line function to find out how to break up the line 
	 * between two points into an "list" of checkpoints
	 */
	public static double[] findNumPoints(double y1, double x1, double y2, double x2){

		double distance, xd, yd, sAngle, xi, yi;
		double numInc;

		//finds difference for x and y
		yd = y2-y1;
		xd = x2-x1;

		//triangle soh cah toa
		sAngle = Math.atan2(yd, xd);
		yi = Math.sin(sAngle)*increment;
		xi = Math.cos(sAngle)*increment;

		//Calculates how many 5mm checkpoints we will divide the line into
		distance = Math.sqrt((xd*xd)+(yd*yd));
		numInc = distance/increment;

		//returns the number of increments the change in x and y for each indivudal point
		double[] iReturn = {numInc, yi, xi};

		return iReturn;

	}

	/*
	 * Solution to exercise 8 part 2
	 * 
	 * Draw a straight line defined by two points.
	 * Finds the final position and uses the line function
	 * 
	 * uses line function from previous exercise 
	 */
	public static void line2(double yS, double xS, double angle, double distance){
		double yF, xF;

		angle = Math.PI*angle/180;

		//triangle soh cah toa
		yF = Math.sin(angle)*distance;
		xF = Math.cos(angle)*distance;

		//uses line function with new coordinates(xF,yF)
		line(yS,xS,yF,xF);


	}

}
