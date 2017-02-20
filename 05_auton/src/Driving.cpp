#include <WPILib.h>
#include <WPILib_auxiliary.h>
#include <Driving.h>
#include <math.h>

Driving::Driving ()
{
	M_fr = new CANTalon(9);
	M_fl = new CANTalon(3);
	M_br = new CANTalon(6);
	M_bl = new CANTalon(7);
    throt = 0.0;
    throt_min = .13;
    throt_rng = 0.87;
    direction_angle = 0;
	nav = new AHRS(I2C::Port::kOnboard);
	nav->Reset();
	imu = new IMU();
	imu->Reset(nav);
	lidar = new LidarLite(I2C::Port::kOnboard,0X62);
}

double Driving::absD (double a) //this may need to be put in a new class. I assumed �Aux_functions�. Only if it is used in other classes.
{
    if(a < 0)
        return -a;
    else
        return a;
}

double Driving::max(double a,double b)
{
	if(a > b)
		return a;
	else
		return b;
}

void Driving::Auto_aim ( double cmxn )
{
    aim_error = -cmxn;

    aim_out = K_aim_p * aim_error + K_aim_i * aim_error_accum + K_aim_d * (aim_error - aim_error_old);
    aim_error_accum += aim_error;

    if(aim_error_accum > MAX_AIM_ACCUM)
        aim_error_accum = MAX_AIM_ACCUM;
    if(aim_error_accum < -MAX_AIM_ACCUM)
        aim_error_accum = -MAX_AIM_ACCUM;

    printf("e: %f\ti = %f\n", aim_error, aim_error_accum);
    aim_error_old = aim_error;

    Move( aim_out , -aim_out , aim_out , -aim_out );  //should turn. should.
}

void Driving::Mecanum_drive(Joystick *control_joy){

	double forward = -1*(control_joy -> GetY());
	double right = control_joy ->GetX();
	double clockwise = control_joy -> GetZ();

	double front_left = forward + clockwise + right;
	double front_right = forward - clockwise - right;
	double rear_left = forward + clockwise - right;
	double rear_right = forward - clockwise + right;

	double max = abs((int)front_left);

	if(abs((int)front_right)>max) max = abs((int)front_right);
	if(abs((int)rear_left)>max) max = abs((int)rear_left);
	if(abs((int)rear_right)>max) max = abs((int)rear_right);

	if(max>1){
		front_left = front_left / max;
		front_right /=max;
		rear_left /=max;
		rear_right /=max;
	}

	Move(front_right, front_left, rear_right, rear_left);

}
void Driving::Manual_driving (Joystick *control_joy)//,ADXRS450_Gyro *gyro)//This doesn't work
{
    con_x = control_joy->GetRawAxis(0);
    con_y = control_joy->GetRawAxis(1);
    con_t = control_joy->GetRawAxis(2);

    scl = absD(con_x) + absD(con_y) + absD(con_t);
    scl = max( scl , 1.0 );

    if(control_joy->GetPOV() != -1.0)
        direction_angle = control_joy->GetPOV();


    throt = ((-control_joy->GetRawAxis(3)+1)/2.0)*throt_rng + throt_min;

	switch( direction_angle )                 //conventional
	{
		case 0 :
			Move(
			(con_y + con_x + con_t) / scl ,
			(con_y - con_x - con_t) / scl ,
			(con_y - con_x + con_t) / scl ,
			(con_y + con_x - con_t) / scl
			);
			break;
		case 90 : //might not work. I just assumed the axes changes. whatever
			Move(
			( -con_y + con_x + con_t) / scl ,
			( +con_y + con_x - con_t) / scl ,
			( +con_y + con_x + con_t) / scl ,
			( -con_y + con_x - con_t) / scl
			);
			break;
		case 180 :
			Move(
			(-con_y - con_x + con_t) / scl ,
			(-con_y + con_x - con_t) / scl ,
			(-con_y + con_x + con_t) / scl ,
			(-con_y - con_x - con_t) / scl
			);
			break;
		case 270 :
			Move(
			( + con_y -con_x + con_t) / scl ,
			( - con_y -con_x - con_t) / scl ,
			( - con_y -con_x + con_t) / scl ,
			( + con_y -con_x - con_t) / scl
			);
			break;
		case -1 :
			//printf(�CHOOSE DIRECTION\n�);
			break;
	}

}

void Driving::Move ( double fr , double fl , double br , double bl )
{
    M_fr->Set( fr * throt );
    M_fl->Set( -fl * throt );
    M_br->Set( br * throt );
    M_bl->Set( -bl * throt );
}

void Driving::Auto_Move ( double fr , double fl , double br , double bl )
{
    M_fr->Set( fr );
    M_fl->Set( -fl );
    M_br->Set( br  );
    M_bl->Set( -bl );
}

void Driving::Auto_Foward (double speed){
	Auto_Move(speed,speed,speed,speed);
}

int Driving::LidarDist(){
	lidar->reset();
	int distance;
	if(lidar->isMeasurementValid(false)){
		distance = lidar->getDistance();
		SmartDashboard::PutNumber("lidar",distance);
		return distance;
	}
	else {
		printf("INVALID DISTANCE");
		return -1;
	}
}


bool Driving::NOPID_Move (int Dist,int Speed)
{
	imu->Localization(nav);
	int lidDist = LidarDist();
	int moveDistance = lidDist*0.9 + imu->position_y*0.1;
	if(moveDistance > Dist){
		Auto_Foward(0.9);
		return true;
	}
	else{
		Auto_Foward(0.0);
		return false;
	}
}

bool Driving::Pid_Move_upTo (double distance)
{

}
