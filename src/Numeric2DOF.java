import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Matrix;

/*
 * Numerical Solution used to solve question 6 using Newton's Method
 * Our analytical Solution was more accurate.
 */
public class Numeric2DOF {
	//initiate motors
	static RegulatedMotor motorFar = new EV3LargeRegulatedMotor(MotorPort.B);
	static RegulatedMotor motorClose = new NXTRegulatedMotor(MotorPort.A);
	//arm lengths in millimeters
	static double armLenC = (8*29);//in mm
	static double armLenF = (8*14);// in mm
	//error that we use to break out of loop in Newton's Method
	static double rError = 10;// in mm

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		NewtonMethod(-100, -100);
	}

	public static void NewtonMethod(double yF, double xF){
		int i;
		double iThetaC = -45;
		double iThetaF = -45;

		//moves to initial position of 0 deg, 90 degs
		motorFar.setSpeed(40);
		motorFar.rotateTo((int)iThetaF, true);
		motorClose.rotateTo((int)iThetaC*7);
		while(motorFar.isMoving());

		//Create Ho vector
		iThetaC = 0*Math.PI/180;
		iThetaF = 90*Math.PI/180;
		double[][] init_points = {{iThetaC},{iThetaF}};
		Matrix iMtrx = new Matrix(init_points);

		//add arm length to x coordinates
		xF += (armLenC+armLenF);

		//loop for newton's method
		for(i = 0; i<= 1000; i++){
			//Create jocobian matrix
			double[][] jocpoints = {{derivative(1,iMtrx.get(0, 0),iMtrx.get(1, 0)),derivative(2,iMtrx.get(0, 0),iMtrx.get(1, 0))},
					{derivative(3,iMtrx.get(0, 0),iMtrx.get(1, 0)), derivative(4,iMtrx.get(0, 0),iMtrx.get(1, 0))}};
			Matrix jocobian = new Matrix(jocpoints);

			//Create f(r)
			double[][] fpoints = {{f(1,iMtrx.get(0, 0),iMtrx.get(1, 0),xF)},
					{f(2,iMtrx.get(0, 0),iMtrx.get(1, 0),yF)}};
			Matrix f_matrix = new Matrix(fpoints);
			
			//Calculate S(i)
			Matrix step = jocobian.inverse().times(f_matrix);
			
			// H(i) = S(i) + H(i-1)
			iMtrx = iMtrx.plus(step);

			//Calculate error for x and y
			double[] error = error_check(iMtrx.get(0, 0)*180/Math.PI,iMtrx.get(1, 0)*180/Math.PI ,yF,xF);

			//break out of the loop if both x and y are less than 10 mm difference
			if (error[0] < rError && error[1] < rError){
				//it converges to a value
				break;
			}
			// if it doesn't solve after 1000 steps quit the function
			if(i == 1000){
				//Function Diverges
				//Reset the motors
				Button.waitForAnyPress();
				motorFar.rotateTo(0);
				motorClose.rotateTo(0);
				return;
			}
		}
		//calculate angles in radiant
		int angleF = (int) (iMtrx.get(1, 0)*180/Math.PI%360);
		int angleC = (int) (iMtrx.get(0, 0)*180/Math.PI%360);

		if(angleF > 180) { angleF -= 360; }
		if(angleC > 180) { angleC -= 360; }
		if(angleF < -180) { angleF += 360; }
		if(angleC < -180) { angleC += 360; }

		//rotates to the final position
		motorFar.rotateTo(angleF,true);
		motorClose.rotateTo(angleC*7);
		while(motorFar.isMoving());

		//Prints angles
		System.out.println("C: " + angleC);
		System.out.println("F: " + angleF);

		//Resets position
		Button.waitForAnyPress();
		motorFar.rotateTo(0);
		motorClose.rotateTo(0);
	}
	
	/*
	 * Helper function for newton's method
	 * Calculates error values used for to decide the accuracy of our generated values
	 */
	private static double[] error_check(double thetaC, double thetaF, double yF, double xF) {
		// TODO Auto-generated method stub
		//Calculates x and y using angles that were generated from our iteration
		double tempx = armLenC * Math.cos(thetaC) + armLenF * Math.cos(thetaC+thetaF);
		double tempy = armLenC * Math.sin(thetaC) + armLenF * Math.sin(thetaC+thetaF);
		
		//Calculates difference between x,y that we were given
		double diffy = Math.abs(yF - tempy);
		double diffx = Math.abs(xF - tempx);
		
		//Return vector of the differences
		double[] soln = {diffy, diffx};
		return soln;
	}
	/*
	 * Helper function for newton's method
	 * Calculates values used for f(r)
	 */
	private static double f(int i, double thetaC,double thetaF, double point) {
		// TODO Auto-generated method stub

		switch(i){
		case 1:
			return armLenC * Math.cos(thetaC) + armLenF * Math.cos(thetaC+thetaF) - point;
		default:
			return armLenC * Math.sin(thetaC) + armLenF * Math.sin(thetaC+thetaF) - point;
		}
	}
	/*
	 * Helper function for newton's method
	 * Calculates values used for jocobian matrix
	 */
	private static double derivative(int i,double thetaC,double thetaF) {
		// TODO Auto-generated method stub

		switch(i){
		case 1:
			return (-1)*(armLenC * Math.sin(thetaC) - armLenF * Math.sin(thetaC+thetaF));
		case 2:
			return (-1)*(-armLenF * Math.sin(thetaC+thetaF));
		case 3:
			return (-1)*(armLenC * Math.cos(thetaC) + armLenF * Math.cos(thetaC+thetaF));
		default:
			return (-1)*(armLenF * Math.cos(thetaC+thetaF));
		}
	}
}
