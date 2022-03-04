#include "main.h"

int macro1 = 975;
int macro2 = 100;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor rmm(3, true);
pros::Motor lmm(11, false);
pros::Motor rbw(6, true);
pros::Motor rfw(4, true);
pros::Motor lfw(5, false);
pros::Motor lbw(13, false);

pros::Motor sixbar(12, true);
int sixbarOffset;

pros::ADIDigitalOut onebar ('A', true);
pros::ADIDigitalOut grabber1 ('B', true);

pros::Imu ins(9);

pros::ADIDigitalIn frontButton ('H');

pros::Motor conveyor(1);

void leftMove(float a1) {
	int a=(int)-a1;
	lmm.move_velocity(-a);
	lfw.move_velocity(a);
	lbw.move_velocity(a);
}

void rightMove(float a1) {
	int a=(int)-a1;
	rmm.move_velocity(-a);
	rfw.move_velocity(a);
	rbw.move_velocity(a);
}


double kP = 1;
double kI = 0;
double kD = 0;

double kP_t = 1;
double kI_t = 0;//.0002;
double kD_t = 0;

// less than 90
double kP_t2 = 0.31;
double kI_t2 = 0;
double kD_t2 = 0;

pros::ADIEncoder re ('E', 'F', true);
pros::ADIEncoder le ('C', 'D');

void pid_turn_relative (float deg, float minspd = 10, float err = 1) {
	double rotOffset = -ins.get_rotation();
	double rot = -ins.get_rotation()-rotOffset;
	double lerror = deg-rot;
	int reOffset = re.get_value();
	int leOffset = le.get_value();
	while (fabs(rot-deg) > err) {
		printf("asdfjkl \n");
		int rv = re.get_value() - reOffset;
		int lv = le.get_value() - leOffset;
		rot = -ins.get_rotation()-rotOffset;
		double error = deg-rot;
		double der = error-lerror;
		lerror = error;
		double move = kP_t*error + der*kD_t;
		// right side pos, ls neg
		if(move > 0) {
			move = std::fmax(move, minspd);
		} else {
			move = std::fmin(move, -minspd);
		}
		rightMove(move);
		leftMove(-move);
		pros::lcd::print(0, "Rotation: %f", rot);
		pros::lcd::print(1, "Goal: %f", deg);
		pros::lcd::print(2, "Error: %f", error);
		pros::lcd::print(3, "Move: %f", move);
		pros::lcd::print(4, "le: %d, re: %d", rv, lv);
		pros::delay(20);
	}

}

void pidTurn (float deg, float max = 127) {
	float distL = (deg/360)*52;
	float distR = -(deg/360)*52;
	le.reset();
	re.reset();
	printf("%f %f", distL, distR);
	float degL = (distL / 12.959) * 360;
	float degR = (distR / 12.959) * 360;

	double errorL = degL;
	double errorR = degR;

	double intL = 0;
	double intR = 0;

	double derL = 2;
	double derR = 2;

	int runs = 0;

	int lposL = -1000000000;
	int lposR = -1000000000;

	while (true) {
		runs += 1;
		int lev = le.get_value();
		int rev = re.get_value();

		if(runs == 110) {
			leftMove(0);
			rightMove(0);
			pros::delay(50);
			break;
		}

		if (fabs(errorL) < 15 && fabs(errorR) < 15) {
			leftMove(0);
			rightMove(0);
			pros::delay(50);
			break;
		}
		lposL = lev;
		lposR = rev;

		errorL = degL - lev;
		errorR = degR - rev;

		intL += errorL;
		intR += errorR;

		derL = lev - lposL;
		derR = rev - lposR;

		if (fabs(errorL) < 10) {
			leftMove(0);
		} else {
			if(deg >= 110) {
				leftMove(fmin(errorL*kP_t + intL*kI_t + derL*kD_t, max));
			} else {
				leftMove(fmin(errorL*kP_t2 + intL*kI_t2 + derL*kD_t2, max));
			}
		}

		if (fabs(errorR) < 10) {
			rightMove(0);
		} else {
			if(deg >= 110) {
				rightMove(fmin(errorR*kP_t + intR*kI_t + derR*kD_t, max));
			} else {
				rightMove(fmin(errorR*kP_t2 + intR*kI_t2 + derR*kD_t2, max));
			}
		}

		printf("%f %f\n", errorL, errorR);
		printf("%d, %d\n", lev, rev);
		printf("----- TURN %d\n", runs);

		pros::lcd::print(0, "%d/%f %d/%f", lev, degL, rev, degR);
		pros::lcd::print(1, "%f, %f", errorL, errorR);
		pros::lcd::print(2, "%f, %f", errorL*kP_t + intL*kI_t + derL*kD_t, errorR*kP_t + intR*kI_t + derR*kD_t);

		pros::delay(20);
	}
}

void pidspec ( float distR, float distL, float max = 127, int limit = 999999999) {
	le.reset();
	re.reset();
	printf("%f %f", distL, distR);
	float degL = (distL / 12.959) * 360;
	float degR = (distR / 12.959) * 360;
	pros::lcd::print(3, "%f, %f", degL, degR);

	double errorL = degL;
	double errorR = degR;

	double intL = 0;
	double intR = 0;

	double derL = 2;
	double derR = 2;

	int runs = 0;

	int lposL = -1000000000;
	int lposR = -1000000000;

	while (true) {
		runs++;
		if(runs == limit) {
			leftMove(0);
			rightMove(0);
			pros::delay(50);
			break;
		}
		int lev = le.get_value();
		int rev = re.get_value();
		printf("%f %f", errorL, errorR);
		if (fabs(errorL) < 40 && fabs(errorR) < 40) {
			leftMove(0);
			rightMove(0);
			pros::delay(50);
			break;
		}

		lposL = lev;
		lposR = rev;

		errorL = degL - lev;
		errorR = degR - rev;

		intL += lev;
		intR += rev;

		derL = lev - lposL;
		derR = rev - lposR;

		double moveL, moveR;
		if (fabs(errorL) < 30) {
		} else {/*
			double move = errorL*kP + intL*kI + derL*kD;
			moveL = fmin(move, max * (move/move));*/
			leftMove(fmin(errorL*kP + intL*kI + derL*kD, max));
		}

		if (fabs(errorR) < 30) {
		} else {/*
			double move = errorR*kP + intR*kI + derR*kD;
			moveR = fmin(move, max * (move/move)); */
			rightMove(fmin(errorL*kP + intL*kI + derL*kD, max));
		}
		printf("%f %f\n", errorL, errorR);
		printf("%d, %d\n", lev, rev);
		printf("----- %d\n", runs);

		pros::lcd::print(0, "%d/%f %d/%f", lev, degL, rev, degR);
		pros::lcd::print(1, "%f, %f, %f", errorL, errorR, max);
		pros::lcd::print(2, "%f, %f", moveL, moveR);

		pros::delay(20);
	}
}

void pid (float dist, float max = 127, int limit = 999999999) {
	pidspec(dist, dist, max, limit);
}

void gotobase (float speed, int maxtime = 999999999) {
	int runs = 0;
	while(!frontButton.get_value()) {
		runs++;
		if(runs == maxtime) {
			break;
		}
		leftMove(speed);
		rightMove(speed);
		pros::delay(10);
	}
	leftMove(0);
	rightMove(0);
}


void initialize() {
		pros::lcd::initialize();

		lbw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		lfw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		rfw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		rbw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		lmm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		rmm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		sixbar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		double motorPos = sixbar.get_position();
		sixbarOffset = motorPos;
		grabber1.set_value(false);
		onebar.set_value(false);

		conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		ins.reset();
		int iter = 0;
		while (ins.is_calibrating()) {
			pros::lcd::print(0, "IMU calibrating... %d\n", iter);
			iter += 10;
			pros::delay(10);
		}
}

void disabled() {}


void competition_initialize() {}


void autonomous() {
	le.reset();
	re.reset();
	printf("alliance goal first then middle, rings");

	leftMove(-127);
	rightMove(-127);
	sixbar.move_velocity(100);
	pros::delay(700);

	leftMove(0);
	rightMove(0);
	pros::delay(200);

	grabber1.set_value(true);
	sixbar.move_velocity(-100);
	pros::delay(200);

	conveyor.move(127);

	pid(2);
	pros::delay(200);

	pid_turn_relative(-70);
	leftMove(0);
	rightMove(0);

	pid(60, 127);
	onebar.set_value(true);
	pros::delay(400);

	pid(-45);
	sixbar.move_velocity(100);
	pros::delay(500);

	pid_turn_relative(-12);
	pros::delay(2000);

	pid(25, 100);

	sixbar.move_velocity(0);
	pid(-25);
	//Turns and moves back and forth for match loads
	//pid_turn_relative(130);
	//onebar.set_value(false);
	//pros::delay(100);

	//pid_turn_relative(150);
	//pid(6, 200);
	//pros::delay(1000);

	//onebar.set_value(true);
	//pros::delay(250);
	//pid(-50);
	//pros::delay(1000);
		//onebar.set_value(true);
		//pid(-14);



	/*while(true) { // enc testing q
		pros::lcd::print(0, "%d %d", le.get_value(), re.get_value());
		printf("%d %d\n", le.get_value(), re.get_value());
		pros::delay(20);
	}*/
	/*
	while(true) {
		pros::lcd::print(0, "%d", frontButton.get_value());
		pros::delay(20);
	}*/


	// TURN WITH 52
	/*
	grabber1.set_value(false);
	leftMove(-127);
	rightMove(-127);
	sixbar.move(40);
	pros::delay(550);
	grabber1.set_value(true);
	pros::delay(90);
	leftMove(0);
	rightMove(0);
	sixbar.move(0);
	onebar.set_value(false);
	grabber1.set_value(true);
	pros::delay(100);
	le.reset();
	re.reset();
	pid(20, -20);
	pros::delay(20);
	conveyor.move(-120);
	pid(90, 50);
	conveyor.move(0);
	pros::delay(20);
	sixbar.move(-127);
	pid(75, 65);
	sixbar.move(0);
	pid(65, 55);
	grabber1.set_value(false);
	pros::delay(200);
	pid(137, 126);
	*/

	/*
	gotobase(127);
	onebar.set_value(true); */
	/*
	conveyor.move(127);
	pros::delay(100);
	grabber1.set_value(false);
	leftMove(-60);
	rightMove(-60);
	sixbar.move(127);
	pros::delay(300);
	grabber1.set_value(true);
	pros::delay(50);
	leftMove(0);
	rightMove(0);
	sixbar.move(0);
	conveyor.move(127);

	//pidspec(0, 30); // turn only using 1 wheel to avoid touching wall, and turn towards the yellow base
	leftMove(80);
	pros::delay(1200);
	leftMove(0);
	sixbar.move(-127);
	gotobase(100); // drive towards the yellow base
	conveyor.move(127);
	sixbar.move(0);
	onebar.set_value(true); // pick up the yellow base
	sixbar.move(127);
	pros::delay(400);
	pidTurn(42);

	pros::delay(100);
	pid(40, 100); //drive all the way to the ramp
	pidTurn(-35);
	pid(8, 127, 250);
	sixbar.move(-127); // slam down on the ramp and wait a bit for the ramp to stabilize
	pros::delay(1500);

	onebar.set_value(false); // relase the base
	sixbar.move(127); // bring the arm up a bit
	pid(-7);
	sixbar.move(0);
	pidTurn(40);
	conveyor.move(127);
	pros::delay(300);


	conveyor.move(127);
	pid(-13); // drive back to the white line
	pidTurn(32); // turn towards the 2 piles of rings
	sixbar.move(-50); // bring the arm down while its at it
	pid(70, 40); // drive slowly into the 2 piles of rings
	pid(-18); // back up a bit so the rings dont get stuck while turning

	//	pros::delay(20000000);


	conveyor.move(127);
	pidTurn(75); // turn into the yello wbase
	gotobase(100); // drive into the yellow base
	onebar.set_value(true); // pick it up
	sixbar.move(0);
	pros::delay(200);

	sixbar.move(127);
	pros::delay(500);
	// DEFAULT 160
	pidTurn(158); // turn towards the red ramp
	pid(44); // dirve twards it
	sixbar.move(0); // drop le base off
	pros::delay(400);
	onebar.set_value(false);
	pros::delay(300);
	sixbar.move(-127);
	pros::delay(100);
	pid(-15); // back up a bit

	grabber1.set_value(false); // release the red base
	pros::delay(200);
	pid(7); // drive forward a bit
	pros::delay(200);
	pidTurn(180); // turn 180
	pros::delay(200);
	gotobase(80); // drive into the base
	onebar.set_value(true);
	sixbar.move(0);
	pros::delay(200);

	sixbar.move(127); // lift le red base
	pidTurn(180);
	pid(33); // drive back into the ramp
	onebar.set_value(false);
	sixbar.move(0);
	pros::delay(400);
	sixbar.move(-127);
	pid(-18);

	// DEFAULT 110
	pidTurn(-135); // turn to the final yellow base

	gotobase(70); //go to teh base
	onebar.set_value(true);
	sixbar.move(127);
	pidTurn(-45);
	pros::delay(200);
	pid(40);
	pros::delay(400);
	sixbar.move(-127);
	pros::delay(800);
	onebar.set_value(false);
	sixbar.move(90);
	pid(-15);
	sixbar.move(0);


	pidTurn(-30); //turn to the final yellow base
	gotobase(100); // drive to it
	onebar.set_value(true); // pick up the yellow base
	sixbar.move(127);
	pros::delay(400);
	pidTurn(42);
	pros::delay(200);
	pid(40); // drive ot the ramp
	onebar.set_value(false); //drop it off
	sixbar.move(-127);
	pid(-20); // back up to dropoff the back
	grabber1.set_value(false);
	pid(5);
	pidTurn(180);
	pid(5);
	onebar.set_value(true); // pcik up the base
	sixbar.move(127);

	pidTurn(180);
	pid(20);
	onebar.set_value(true);
*/




}

void opcontrol() {
	//autonomous();


		// PORTS AND INTERFACE SETUP
		pros::Controller master(pros::E_CONTROLLER_MASTER);

		lbw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		lfw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		rfw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		rbw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		lmm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		rmm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

		//pros::Motor grabber(16);
		//grabber.set_brake_mode(pro    s::E_MOTOR_BRAKE_HOLD);


		bool one_bar = false;
		bool grab_ber = false;
		bool grab_bers = false;

		bool lone_bar = false;

		bool lhold = false;
		bool chold = false;

		while (true){

				int left = master.get_analog(ANALOG_LEFT_Y);
				int right = master.get_analog(ANALOG_RIGHT_Y);
				int moveL = -left;
				int moveR = -right;


				lbw.move(moveL);
				lfw.move(moveL);
				lmm.move(-moveL);
				rmm.move(-moveR);
				rbw.move(moveR);
				rfw.move(moveR);


				bool MOTOR_HOLD = master.get_digital(DIGITAL_X);
				if (!lhold && MOTOR_HOLD){
					chold = !chold;
					pros::lcd::print(1, "%d", chold);

					if (chold) {
						lbw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
						lfw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
						rfw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
						rbw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
						lmm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
						rmm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
					} else {
						lbw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
						lfw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
						rfw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
						rbw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
						lmm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
						rmm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
					}
				}
				lhold = MOTOR_HOLD;

				bool CONVEYOR_UP = master.get_digital(DIGITAL_L1);
				bool CONVEYOR_DOWN = master.get_digital(DIGITAL_L2);
				bool CONVEYOR_STOP = master.get_digital(DIGITAL_Y);

				if (CONVEYOR_UP){
						conveyor.move(127);
				} else if (CONVEYOR_DOWN){
						conveyor.move(-127);
				} else if (CONVEYOR_STOP){
						conveyor.move(0);
				}

				bool ONE_BAR_TOGGLE = master.get_digital(DIGITAL_A);
				if (!lone_bar && ONE_BAR_TOGGLE){
						onebar.set_value(one_bar);
						one_bar = !(one_bar);
				}
				lone_bar = ONE_BAR_TOGGLE;

				bool GRAB_BER_TOGGLE = master.get_digital(DIGITAL_B);
				if (!grab_bers && GRAB_BER_TOGGLE){
						grabber1.set_value(grab_ber);
						grab_ber = !(grab_ber);
				}
				grab_bers = GRAB_BER_TOGGLE;
				bool SIXBAR_UP = master.get_digital(DIGITAL_R1);
				bool SIXBAR_DOWN = master.get_digital(DIGITAL_R2);

				if (SIXBAR_UP){
						sixbar.move(127);
				}else if (SIXBAR_DOWN){
						sixbar.move(-127);
				}else{
						// KEYBINDS
						bool SIXBAR_MACRO_1 = master.get_digital(DIGITAL_DOWN);
						bool SIXBAR_MACRO_2 = master.get_digital(DIGITAL_UP);
						if (SIXBAR_MACRO_1) {
							sixbar.move_absolute(macro1 + sixbarOffset, 127);
							pros::lcd::print(0, "Goal: %d Current: %f", macro1+sixbarOffset, sixbar.get_position());
						} else if (SIXBAR_MACRO_2) {
							sixbar.move_absolute(macro2 + sixbarOffset, 127);
							pros::lcd::print(1, "Goal: %d Current: %f", macro2+sixbarOffset, sixbar.get_position());
						} else {
							sixbar.move(0);
						}
				}
				pros::delay(20);
		}
}
