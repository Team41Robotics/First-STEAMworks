#include <iostream>
#include <memory>
#include <string>
#include <WPIlib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Driving.h>
#include <IMU.h>
#include <WPILib_auxiliary.h>
#include <Shooter.h>



class Robot: public frc::IterativeRobot {
public:
	Driving *motion_control;
	Shooter *shooter;
	IMU *imu;
	Joystick *control_0;
	AHRS *nav;
	Timer *timer;
	double time = 0.0;

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		motion_control = new Driving();
		shooter = new Shooter();
		control_0 = new Joystick(0);
		timer = new Timer();
		timer->Start();
		nav = new AHRS(I2C::Port::kOnboard);
		nav->Reset();
		imu = new IMU();
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


	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
			SmartDashboard::PutString("Auton","doop");
		} else {
			// Default Auto goes here
			SmartDashboard::PutString("Auton","preston");
		}
		timer->Reset();
		timer->Start();

	}
	void AutonomousPeriodic() {
		time = timer->Get();
		if (autoSelected == autoNameCustom) {
			imu->Localization(nav);
			if(imu->position_y < 114)
			{
				motion_control->Auto_Move(-0.25,-0.25,-0.25,-0.25);
			}
			else if(imu->theta < 30)
			{
				motion_control->Auto_Move(+0.25,-0.25,+0.25,-0.25);
			}
			else if(imu->position_x < 12.5)
			{
				motion_control->Auto_Move(-0.50,-0.50,-0.50,-0.50);;
			}

		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		motion_control->Manual_driving(control_0);

		imu->Localization(nav);
		SmartDashboard::PutNumber("X", imu->position_x);
		SmartDashboard::PutNumber("Y", imu->position_y);
		SmartDashboard::PutNumber("vel x", imu->velocity_x);
		SmartDashboard::PutNumber("vel Y", imu->velocity_y);
		SmartDashboard::PutNumber("accel Y", imu->accel_y);
		SmartDashboard::PutNumber("accel X", imu->accel_x);

		//SmartDashboard::PutNumber("D Y", nav->GetDisplacementX());

		if(control_0->GetRawButton(12))
		{
			nav->ZeroYaw();
			nav->ResetDisplacement();
		}
		//imu->Localization(nav);
		//SmartDashboard::PutNumber("X",imu->position_x);
		//SmartDashboard::PutNumber("Y",imu->position_y);
		//SmartDashboard::PutNumber("R",imu->position_r);
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