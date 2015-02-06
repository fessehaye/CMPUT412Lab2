import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;


public class pathplanning {

    static double armLenC = (8*29);//in mm
    static double armLenF = (8*14);// in mm
    static double increment = 5;// in mm
	static RegulatedMotor motorFar = new EV3LargeRegulatedMotor(MotorPort.B);
	static RegulatedMotor motorClose = new NXTRegulatedMotor(MotorPort.A);
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub
//		line(0,0,0,-100);
//		line2(0,0, 135, 100);
//		double[][] points = {
//				{0,0},
//				{100,-50},
//				{100,-75}
//				};
//		arc(points);
		labyrinth();
	}

	private static void labyrinth() {
		motorFar.setSpeed(40);
		//list movements
//		line(0,0,40,-200);

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
		arc(points);
		Button.waitForAnyPress();
		motorFar.rotateTo(0);
		motorClose.rotateTo(0);
		
	}

	private static void arc(double[][] points) {
		// TODO Auto-generated method stub
		for(int i = 0; i<points.length - 1; i++){
			System.out.print(points[i][0]);
			System.out.println(points[i][1]);
			
			System.out.print(points[i+1][0]);
			System.out.println(points[i+1][1]);
			line(points[i][0],points[i][1],points[i+1][0],points[i+1][1]);
//			Delay.msDelay(5000);
		}
	}

	public static void line(double yS, double xS, double yF, double xF){
		double[] cS = {yS,xS};
		double[] cF = {yF,xF};
		double xi = 0;
		double yi = 0; 
		int[] thetas;
		
		double[] inc = findNumPoints(cS[0],cS[1],cF[0],cF[1]);
		
		thetas = getThetas4Position(cS[0], cS[1]);
		motorClose.rotateTo(thetas[0],true);
		motorFar.rotateTo(thetas[1]); 
		
		while(motorClose.isMoving());
		
		for(int i=0; i<inc[0]; i++){
			yi += inc[1];
			xi += inc[2];
			
			thetas = getThetas4Position(cS[0]+yi, cS[1]+xi);
			
			motorClose.rotateTo(thetas[0],true);
			motorFar.rotateTo(thetas[1]); 
			
			while(motorClose.isMoving());
//			Delay.msDelay(200);
		}
		
		thetas = getThetas4Position(cF[0], cF[1]);
		motorClose.rotateTo(thetas[0],true);
		motorFar.rotateTo(thetas[1]); 
		
		while(motorClose.isMoving()); 
	}
	
	
	
	
	public static int[] getThetas4Position(double y, double x) {
		double d, theta1, theta2;
		
		x += (armLenC+armLenF);
		
		d = ((x*x)+(y*y)-(armLenC*armLenC)-(armLenF*armLenF))/(2*armLenC*armLenF);
		
		theta2 = Math.atan2(Math.sqrt(1-(d*d)), d);
		theta1 = Math.atan2(y, x) - Math.atan2(armLenF*Math.sin(theta2),(armLenC+(armLenF*Math.cos(theta2))));
		
		theta1 = theta1*180/Math.PI;
		theta2 = theta2*180/Math.PI;

		//remember when moving to add *7 to theta1
		int[] solution = {(int)theta1*7,(int)theta2};
		
		return solution;
	}

	
	public static double[] findNumPoints(double y1, double x1, double y2, double x2){

		double distance, xd, yd, sAngle, xi, yi;
		double numInc;

		yd = y2-y1;
		xd = x2-x1;
		
		//triangle soh cah toa
		sAngle = Math.atan2(yd, xd);
		yi = Math.sin(sAngle)*increment;
		xi = Math.cos(sAngle)*increment;
		
		distance = Math.sqrt((xd*xd)+(yd*yd));
		
		numInc = distance/increment;
		
		double[] iReturn = {numInc, yi, xi};
		
		return iReturn;
		
	}
	
	public static void line2(double yS, double xS, double angle, double distance){
		double yF, xF;
		
		angle = Math.PI*angle/180;
		
		//triangle soh cah toa
		yF = Math.sin(angle)*distance;
		xF = Math.cos(angle)*distance;
		

		System.out.print("d: " + distance + "mm");
		System.out.println("@: " + (int)180*angle/Math.PI);
		
		line(yS,xS,yF,xF);

		
	}
	
}
