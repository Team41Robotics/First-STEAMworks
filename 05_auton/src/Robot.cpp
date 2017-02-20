#include <iostream>
#include <memory>
#include <string>
#include <WPIlib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Driving.h>
#include <WPILib_auxiliary.h>
#include <Shooter.h>

class Robot: public frc::IterativeRobot {
public:
	Driving *motion_control;
	Shooter *shooter;
	Joystick *control_0;
	Timer *timer;

	double time = 0.0;
	int auton_step = 0;


	void RobotInit() {
		chooser.AddDefault(left, left);
		chooser.AddObject(middle, middle);
		chooser.AddObject(right, right);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		color.AddDefault(blue, blue);
		color.AddObject(red, red);
		frc::SmartDashboard::PutData("color Modes", &color);


		motion_control = new Driving();
		shooter = new Shooter();
		control_0 = new Joystick(0);
		timer = new Timer();
		timer->Start();

	}

	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (colorSelected == blue) {
			SmartDashboard::PutString("Auton","left");
		}
		else if (autoSelected == middle) {
			SmartDashboard::PutString("Auton","middle");
		}
		else if (autoSelected == right){
			SmartDashboard::PutString("Auton","right");
		}

		colorSelected = color.GetSelected();
		std::cout << "Color selected: " << colorSelected << std::endl;

		if (colorSelected == blue) {
			SmartDashboard::PutString("Color","Blue");
		}
		else if (colorSelected == red) {
			SmartDashboard::PutString("Color","Red");
		}


		timer->Reset();
		timer->Start();
		auton_step = 0;
	}
	void AutonomousPeriodic() {
		time = timer->Get();
		if (autoSelected == left) {

			if(auton_step == 0)
			{
/*				motion_control->Auto_Move(-0.5,-0.5,-0.5,-0.5);
				if(imu->position_y < 135.4)*/
				//if(motion_control->NOPID_Move)
			//		auton_step = 1;
			if(motion_control->Pid_Move_Upto(135.4))
				auton_step = 1;

			}
			else if(auton_step == 1)
			{
/*				imu->ResetPos();
				motion_control->Auto_Move(+0.25,-0.25,+0.25,-0.25);
				if(imu->theta < 90)*/
					auton_step = 2;
				//TURN FUNC
			}
			else if(auton_step == 2)
			{
/*				motion_control->Auto_Move(-0.50,-0.50,-0.50,-0.50);
				if(imu->position_y <  0.0)*/
					auton_step = 3;
			}
			else
			{
				motion_control->Auto_Move(0,0,0,0);
			}


		}/* else {
			time = timer->Get();
			imu->Localization(nav);
			SmartDashboard::PutNumber("distance",imu->position_y);
			if(imu->position_y < 1.00)
			{

				motion_control->Auto_Move(-0.5,-0.5,-0.5,-0.5);
			}
		}*/
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
/*		motion_control->Manual_driving(control_0);



		imu->Localization(nav);

		SmartDashboard::PutNumber("raw Accel",nav->GetRawAccelY());
		SmartDashboard::PutNumber("raw Accelx",nav->GetRawAccelX());
		SmartDashboard::PutNumber("raw gyroz",nav->GetRawGyroZ());
		SmartDashboard::PutNumber("raw gyrox",nav->GetRawGyroX());
		SmartDashboard::PutNumber("raw gyroy",nav->GetRawGyroY());
		SmartDashboard::PutNumber("velocity ",nav->GetVelocityY());

		SmartDashboard::PutNumber("X", imu->position_x);
		SmartDashboard::PutNumber("Y", imu->position_y);
		SmartDashboard::PutNumber("vel x", imu->velocity_x);
		SmartDashboard::PutNumber("vel Y", imu->velocity_y);
		SmartDashboard::PutNumber("accel Y", imu->accel_y);
		SmartDashboard::PutNumber("accel X", imu->accel_x);

		//SmartDashboard::PutNumber("D Y", nav->GetDisplacementX());

		if(control_0->GetRawButton(12))
		{
			imu->Reset(nav);
		}
		//imu->Localization(nav);
		//SmartDashboard::PutNumber("X",imu->position_x);
		//SmartDashboard::PutNumber("Y",imu->position_y);
		//SmartDashboard::PutNumber("R",imu->position_r);*/
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string left = "Left";
	const std::string middle = "Middle";
	const std::string right = "Right";


	frc::SendableChooser<std::string> color;
	const std::string blue = "Blue";
	const std::string red = "Red";


	std::string autoSelected;
	std::string colorSelected;
};

START_ROBOT_CLASS(Robot)
