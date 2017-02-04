#include <iostream>
#include <memory>
#include <string>
#include <WPIlib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Driving.h>

#define G 9.80665
#define RAD 3.14159265359 / 180.0

#define K_a0 0.5
#define K_a1 0.5

class Robot: public frc::IterativeRobot {
public:
	Driving *motion_control;
	Joystick *control_0;
	BuiltInAccelerometer *accel_0;
	ADXL345_I2C *accel_1;

	ADXRS450_Gyro *gyro_0;

	Timer *timer;

	double curr_t =0.0;
	double old_t =0.0;
	double del_t =0.0;

	double theta = 0.0;
	double accel_x =0.0;
	double accel_y =0.0;
	double velocity_x =0.0;
	double velocity_y =0.0;
	double position_x =0.0;
	double position_y =0.0;

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		motion_control = new Driving();
		control_0 = new Joystick(0);
		//2 accelerometers because they both suck so maybe their sucking can interfere into no sucking?
		accel_0 = new BuiltInAccelerometer();
		accel_1 = new ADXL345_I2C(I2C::Port::kOnboard, Accelerometer::Range::kRange_4G);
		gyro_0 = new ADXRS450_Gyro();
		gyro_0->Calibrate();

		timer = new Timer();
		timer->Reset();
		timer->Start();
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */

	void Get_delta_time ()
	{
		curr_t = timer->Get();    //unsure what units. need it in seconds.
		del_t = curr_t - old_t;
		old_t = curr_t;
	}

	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		//put shooter shit stuff here. (ponzio is watching)
		//should be easy enough to do. Just setting it. The freshmen were tasked with this. They should have that stuff. I’ll add when we have the shooter.

		//oh, gee, what else do they want?

		motion_control->Manual_driving(control_0);

		Get_delta_time();

		accel_x = ( accel_0->GetX() * K_a0 + accel_1->GetX() * K_a1 ) * G;
		// K_a are accelerometer constants. Ask Neelay. He should know a good value. Also see him regarding iterative filtering to reduce error.
		accel_y = ( accel_0->GetY() * K_a0 + accel_1->GetY() * K_a1 ) * G;

		theta = gyro_0->GetAngle() * RAD;

		velocity_x += ( accel_x * cos(theta) + accel_y * sin(theta) ) * del_t;
		velocity_y += ( accel_y * cos(theta) + accel_x * sin(theta) ) * del_t;

		position_x += velocity_x * del_t;//very high in accError
		position_y += velocity_y * del_t;
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
