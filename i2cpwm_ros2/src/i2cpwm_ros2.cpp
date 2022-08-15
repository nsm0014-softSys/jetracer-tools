
#include <i2cpwm_ros2.hpp>

// Class implementations
void i2cpwm_ros2::_i2cpwm_ros2_setup()
{
	// Set logging string
	this->_logging_level_str = this->get_parameter("logger_level").as_string();
	// handle logger setup
	this->handle_logger_level();

	// Get drive mode details from params
	this->_active_drive.mode = this->get_parameter("drive_config.mode").as_int();
	this->_active_drive.rpm = this->get_parameter("drive_config.rpm").as_double();
	this->_active_drive.radius = this->get_parameter("drive_config.radius").as_double();
	this->_active_drive.track = this->get_parameter("drive_config.track").as_double();
	this->_active_drive.scale = this->get_parameter("drive_config.scale").as_double();

		// get i2c device info and setup connnection
	this->_device = this->get_parameter("i2c_device_address").as_string()
				  + std::to_string(this->_controller_io_device);
	RCLCPP_INFO(rclcpp::get_logger("i2cpwm_ros2"), "Setup: %s", this->_device);

	this->_init(this->_device.c_str());
	this->_set_active_board(1);
	this->_set_pwm_frequency(this->_pwm_frequency);
	int id = this->get_parameter("drive_config.servos.servo1.number").as_int();
	int pos = this->get_parameter("drive_config.servos.servo1.position").as_int();
	this->_config_servo_position(id, pos);
}

float i2cpwm_ros2::_abs (float v1)
{
	if (v1 < 0)
		return (0 - v1);
	return v1;
}

float i2cpwm_ros2::_min (float v1, float v2)
{
	if (v1 > v2)
		return v2;
	return v1;
}

float i2cpwm_ros2::_max (float v1, float v2)
{
	if (v1 < v2)
		return v2;
	return v1;
}

float i2cpwm_ros2::_absmin (float v1, float v2)
{
	float a1, a2;
	float sign = 1.0;
	//	if (v1 < 0)
	//		sign = -1.0;
	a1 = this->_abs(v1);
	a2 = this->_abs(v2);
	if (a1 > a2)
		return (sign * a2);
	return v1;
}

float i2cpwm_ros2::_absmax (float v1, float v2)
{
	float a1, a2;
	float sign = 1.0;
	//	if (v1 < 0)
	//		sign = -1.0;
	a1 = this->_abs(v1);
	a2 = this->_abs(v2);
	if (a1 < a2)
		return (sign * a2);
	return v1;
}

int i2cpwm_ros2::_smoothing (float speed)
{
	/* if smoothing is desired, then remove the commented code  */
	// speed = (cos(_PI*(((float)1.0 - speed))) + 1) / 2;
	return speed;
}


float i2cpwm_ros2::_convert_mps_to_proportional (float speed)
{
	/* we use the drive mouter output rpm and wheel radius to compute the conversion */

	float initial, max_rate;	// the max m/s is ((rpm/60) * (2*PI*radius))

	initial = speed;
	
	if (this->_active_drive.rpm <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid active drive mode RPM %6.4f :: RPM must be greater than 0", this->_active_drive.rpm);
		return 0.0;
	}
	if (this->_active_drive.radius <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid active drive mode radius %6.4f :: wheel radius must be greater than 0", this->_active_drive.radius);
		return 0.0;
	}

	max_rate = (this->_active_drive.radius * _PI * 2) * (this->_active_drive.rpm / 60.0);

	speed = speed / max_rate;
	// speed = _absmin (speed, 1.0);

	RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"%6.4f = convert_mps_to_proportional ( speed(%6.4f) / ((radus(%6.4f) * pi(%6.4f) * 2) * (rpm(%6.4f) / 60.0)) )", speed, initial, this->_active_drive.radius, _PI, _active_drive.rpm);
	return speed;
}

void i2cpwm_ros2::_set_pwm_frequency (int freq)
{
    int prescale;
    char oldmode, newmode;
    int res;

    this->_pwm_frequency = freq;   // save to global
    
	RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"_set_pwm_frequency prescale");
    float prescaleval = 25000000.0; // 25MHz
    prescaleval /= 4096.0;
    prescaleval /= (float)freq;
    prescaleval -= 1.0;
    //RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Estimated pre-scale: %6.4f", prescaleval);
    prescale = floor(prescaleval + 0.5);
    // RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Final pre-scale: %d", prescale);


	RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Setting PWM frequency to %d Hz", freq);

    nanosleep ((const struct timespec[]){{1, 000000L}}, NULL); 


    oldmode = i2c_smbus_read_byte_data (this->_controller_io_handle, __MODE1);
    newmode = (oldmode & 0x7F) | 0x10; // sleep

    if (0 > i2c_smbus_write_byte_data (this->_controller_io_handle, __MODE1, newmode)) // go to sleep
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Unable to set PWM controller to sleep mode"); 

    if (0 >  i2c_smbus_write_byte_data(this->_controller_io_handle, __PRESCALE, (int)(floor(prescale))))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Unable to set PWM controller prescale"); 

    if (0 > i2c_smbus_write_byte_data(this->_controller_io_handle, __MODE1, oldmode))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Unable to set PWM controller to active mode"); 

    nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec,

    if (0 > i2c_smbus_write_byte_data(this->_controller_io_handle, __MODE1, oldmode | 0x80))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Unable to restore PWM controller to active mode");
}

void i2cpwm_ros2::_set_pwm_interval_all (int start, int end)
{
    // the public API is ONE based and hardware is ZERO based
    if ((this->_active_board<1) || (this->_active_board>62)) {
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Internal error - invalid active board number %d :: PWM board numbers must be between 1 and 62", _active_board);
        return;
    }
    int board = this->_active_board - 1;

    if (0 > i2c_smbus_write_byte_data (this->_controller_io_handle, __ALL_CHANNELS_ON_L, start & 0xFF))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Error setting PWM start low byte for all servos on board %d", this->_active_board);
    if (0 >  i2c_smbus_write_byte_data (this->_controller_io_handle, __ALL_CHANNELS_ON_H, start  >> 8))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Error setting PWM start high byte for all servos on board %d", this->_active_board);
    if (0 > i2c_smbus_write_byte_data (this->_controller_io_handle, __ALL_CHANNELS_OFF_L, end & 0xFF))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Error setting PWM end low byte for all servos on board %d", this->_active_board);
    if (0 > i2c_smbus_write_byte_data (this->_controller_io_handle, __ALL_CHANNELS_OFF_H, end >> 8))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Error setting PWM end high byte for all servos on board %d", this->_active_board);
}

void i2cpwm_ros2::_set_active_board (int board)
{
	char mode1res;

	if ((board<1) || (board>62)) {
        RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Internal error :: invalid board number %d :: board numbers must be between 1 and 62", board);
        return;
    }
    if (this->_active_board != board) {
        this->_active_board = board;   // save to global
        
        // the public API is ONE based and hardware is ZERO based
        board--;
        
        if (0 > ioctl (this->_controller_io_handle, I2C_SLAVE, (_BASE_ADDR+(board)))) {
            RCLCPP_FATAL(rclcpp::get_logger("i2cpwm_ros2"), "Failed to acquire bus access and/or talk to I2C slave at address 0x%02X", (_BASE_ADDR+board));
            return; /* exit(1) */   /* additional ERROR HANDLING information is available with 'errno' */
        }

        if (_pwm_boards[board]<0) {
            _pwm_boards[board] = 1;

            /* this is guess but I believe the following needs to be done on each board only once */

            if (0 > i2c_smbus_write_byte_data (this->_controller_io_handle, __MODE2, __OUTDRV))
                RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Failed to enable PWM outputs for totem-pole structure");

            if (0 > i2c_smbus_write_byte_data (this->_controller_io_handle, __MODE1, __ALLCALL))
                RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Failed to enable ALLCALL for PWM channels");

            nanosleep ((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci


            mode1res = i2c_smbus_read_byte_data (this->_controller_io_handle, __MODE1);
            mode1res = mode1res & ~__SLEEP; //                 # wake up (reset sleep)

            if (0 > i2c_smbus_write_byte_data (this->_controller_io_handle, __MODE1, mode1res))
                RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Failed to recover from low power mode");

            nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci

            // the first time we activate a board, we mark it and set all of its servo channels to 0
            this->_set_pwm_interval_all (0, 0);
        }
    }
}

void i2cpwm_ros2::_set_pwm_interval (int servo, int start, int end)
{
	RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"_set_pwm_interval enter");

    if ((servo<1) || (servo>(MAX_SERVOS))) {
        RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid servo number %d :: servo numbers must be between 1 and %d", servo, MAX_BOARDS);
        return;
    }

	int board = ((int)((servo-1)/16))+1;	// servo 1..16 is board #1, servo 17..32 is board #2, etc.
	this->_set_active_board(board);

	servo = ((servo-1) % 16) + 1;			// servo numbers are 1..16


    // the public API is ONE based and hardware is ZERO based
    board = this->_active_board - 1;				// the hardware enumerates boards as 0..61
    int channel = servo - 1;				// the hardware enumerates servos as 0..15
	RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"_set_pwm_interval board=%d servo=%d", board, servo);
    
    if (0 > i2c_smbus_write_byte_data (this->_controller_io_handle, __CHANNEL_ON_L+4*channel, start & 0xFF))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Error setting PWM start low byte on servo %d on board %d", servo, this->_active_board);
    if (0 >  i2c_smbus_write_byte_data (this->_controller_io_handle, __CHANNEL_ON_H+4*channel, start  >> 8))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Error setting PWM start high byte on servo %d on board %d", servo, this->_active_board);
    if (0 > i2c_smbus_write_byte_data (this->_controller_io_handle, __CHANNEL_OFF_L+4*channel, end & 0xFF))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Error setting PWM end low byte on servo %d on board %d", servo, this->_active_board);
    if (0 > i2c_smbus_write_byte_data (this->_controller_io_handle, __CHANNEL_OFF_H+4*channel, end >> 8))
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Error setting PWM end high byte on servo %d on board %d", servo, this->_active_board);
}


// TODO: NEEDS WORK
void i2cpwm_ros2::_set_pwm_interval_proportional (int servo, float value)
{
	// need a little wiggle room to allow for accuracy of a floating point value
	if ((value < -1.0001) || (value > 1.0001)) {
		RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Invalid proportion value %f :: proportion values must be between -1.0 and 1.0", value);
		return;
	}

	servo_config* configp = &(_servo_configs[servo-1]);
	
	if ((configp->center < 0) ||(configp->range < 0)) {
		RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Missing servo configuration for servo[%d]", servo);
		return;
	}

	int pos = (configp->direction * (((float)(configp->range) / 2) * value)) + configp->center;
        
	if ((pos < 0) || (pos > 4096)) {
		RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"Invalid computed position servo[%d] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d", servo, configp->direction, configp->range, value, configp->center, pos);
		return;
	}
	this->_set_pwm_interval (servo, 0, pos);
	RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"servo[%d] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d", servo, configp->direction, configp->range, value, configp->center, pos);
}

void i2cpwm_ros2::_config_servo (int servo, int center, int range, int direction)
{
	if ((servo < 1) || (servo > (MAX_SERVOS))) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid servo number %d :: servo numbers must be between 1 and %d", servo, MAX_SERVOS);
		return;
	}

	if ((center < 0) || (center > 4096))
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid center value %d :: center values must be between 0 and 4096", center);

	if ((center < 0) || (center > 4096))
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid range value %d :: range values must be between 0 and 4096", range);

	if (((center - (range/2)) < 0) || (((range/2) + center) > 4096))
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid range center combination %d ± %d :: range/2 ± center must be between 0 and 4096", center, (range/2));

	this->_servo_configs[servo-1].center = center;
	this->_servo_configs[servo-1].range = range;
	this->_servo_configs[servo-1].direction = direction;
	// _servo_configs[servo-1].mode_pos = POSITION_UNDEFINED;

	if (servo > this->_last_servo)	// used for internal optimizations
		this->_last_servo = servo;

	RCLCPP_INFO(rclcpp::get_logger("i2cpwm_ros2"),"Servo #%d configured: center=%d, range=%d, direction=%d", servo, center, range, direction);
}

int i2cpwm_ros2::_config_servo_position (int servo, int position)
{
	if ((servo < 1) || (servo > (MAX_SERVOS))) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid servo number %d :: servo numbers must be between 1 and %d", servo, MAX_SERVOS);
		return -1;
	}
	if ((position < POSITION_UNDEFINED) || (position > POSITION_RIGHTREAR)) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid drive mode position %d :: positions are 0 = non-drive, 1 = left front, 2 = right front, 3 = left rear, and 4 = right rear", position);
		return -1;
	}
	this->_servo_configs[servo-1].mode_pos = position;
	RCLCPP_INFO(rclcpp::get_logger("i2cpwm_ros2"),"Servo #%d configured: position=%d", servo, position);
	return 0;
}

int i2cpwm_ros2::_config_drive_mode (std::string mode, float rpm, float radius, float track, float scale)
{
	int mode_val = MODE_UNDEFINED;

	// assumes the parameter was provided in the proper case
	if 		(0 == strcmp (mode.c_str(), _CONST("ackerman")))
		mode_val = MODE_ACKERMAN;
	else if (0 == strcmp (mode.c_str(), _CONST("differential")))
		mode_val = MODE_DIFFERENTIAL;
	else if (0 == strcmp (mode.c_str(), _CONST("mecanum")))
		mode_val = MODE_MECANUM;
	else {
		mode_val = MODE_INVALID;
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid drive mode %s :: drive mode must be one of ackerman, differential, or mecanum", mode.c_str());
		return -1;
	}

	if (rpm <= 0.0) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid RPM %6.4f :: the motor's output RPM must be greater than 0.0", rpm);
		return -1;
	}

	if (radius <= 0.0) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid radius %6.4f :: the wheel radius must be greater than 0.0 meters", radius);
		return -1;
	}

	if (track <= 0.0) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid track %6.4f :: the axel track must be greater than 0.0 meters", track);
		return -1;
	}

	if (scale <= 0.0) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid scale %6.4f :: the scalar for Twist messages must be greater than 0.0", scale);
		return -1;
	}

	this->_active_drive.mode = mode_val;
	this->_active_drive.rpm = rpm;
	this->_active_drive.radius = radius;	// the service takes the radius in meters
	this->_active_drive.track = track;		// the service takes the track in meters
	this->_active_drive.scale = scale;

	RCLCPP_INFO(rclcpp::get_logger("i2cpwm_ros2"),"Drive mode configured: mode=%s, rpm=%6.4f, radius=%6.4f, track=%6.4f, scale=%6.4f", mode.c_str(), rpm, radius, track, scale);
	return 0;
}

void i2cpwm_ros2::_init (const char* filename)
{
    int res;
    char mode1res;
    int i;

    /* initialize all of the global data objects */
    
    for (i=0; i<MAX_BOARDS;i++)
        this->_pwm_boards[i] = -1;
    this->_active_board = -1;

	for (i=0; i<(MAX_SERVOS);i++) {
		// these values have not useful meaning
		this->_servo_configs[i].center = -1;
		this->_servo_configs[i].range = -1;
		this->_servo_configs[i].direction = 1;
		this->_servo_configs[i].mode_pos = -1;
	}
	this->_last_servo = -1;

	this->_active_drive.mode = MODE_UNDEFINED;
	this->_active_drive.rpm = -1.0;
	this->_active_drive.radius = -1.0;
	this->_active_drive.track = -1.0;
	this->_active_drive.scale = -1.0;
	
	
    if ((this->_controller_io_handle = open (filename, O_RDWR)) < 0) {
        RCLCPP_FATAL(rclcpp::get_logger("i2cpwm_ros2"), "Failed to open I2C bus %s", filename);
        return; /* exit(1) */   /* additional ERROR HANDLING information is available with 'errno' */
    }
	RCLCPP_INFO(rclcpp::get_logger("i2cpwm_ros2"), "I2C bus opened on %s", filename);
}


void i2cpwm_ros2::servos_absolute (const i2cpwm_ros2_msgs::msg::ServoArray::SharedPtr msg)
{
    /* this subscription works on the active_board */
    
    for(std::vector<i2cpwm_ros2_msgs::msg::Servo>::const_iterator sp = msg->servos.begin(); sp != msg->servos.end(); ++sp) {
        int servo = sp->servo;
        int value = sp->value;

        if ((value < 0) || (value > 4096)) {
            RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid PWM value %d :: PWM values must be between 0 and 4096", value);
            continue;
        }
        this->_set_pwm_interval (servo, 0, value);
        RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"servo[%d] = %d", servo, value);
    }
}

void i2cpwm_ros2::servos_proportional (const i2cpwm_ros2_msgs::msg::ServoArray::SharedPtr msg)
{
    /* this subscription works on the active_board */

    for(std::vector<i2cpwm_ros2_msgs::msg::Servo>::const_iterator sp = msg->servos.begin(); sp != msg->servos.end(); ++sp) {
        int servo = sp->servo;
        float value = sp->value;
		this->_set_pwm_interval_proportional (servo, value);
    }
}

void i2cpwm_ros2::servos_drive (const geometry_msgs::msg::Twist::SharedPtr msg)
{
	/* this subscription works on the active_board */

	int i;
	float delta, range, ratio;
	float temp_x, temp_y, temp_r;
	float dir_x, dir_y, dir_r;
	float speed[4];
	
	/* msg is a pointer to a Twist message: msg->linear and msg->angular each of which have members .x .y .z */
	/* the subscriber uses the maths from: http://robotsforroboticists.com/drive-kinematics/ */	

	RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"servos_drive Twist = [%5.2f %5.2f %5.2f] [%5.2f %5.2f %5.2f]", 
			 msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);

	if (this->_active_drive.mode == MODE_UNDEFINED) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"drive mode not set");
		return;
	}
	if ((this->_active_drive.mode < MODE_UNDEFINED) || (this->_active_drive.mode >= MODE_INVALID)) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"unrecognized drive mode set %d", this->_active_drive.mode);
		return;
	}

	dir_x = ((msg->linear.x  < 0) ? -1 : 1);
	dir_y = ((msg->linear.y  < 0) ? -1 : 1);
	dir_r = ((msg->angular.z < 0) ? -1 : 1);

	temp_x = this->_active_drive.scale * this->_abs(msg->linear.x);
	temp_y = this->_active_drive.scale * this->_abs(msg->linear.y);
	temp_r = this->_abs(msg->angular.z);	// radians
		
	// temp_x = _smoothing (temp_x);
	// temp_y = _smoothing (temp_y);
	// temp_r = _smoothing (temp_r) / 2;

	// the differential rate is the robot rotational circumference / angular velocity
	// since the differential rate is applied to both sides in opposite amounts it is halved
	delta = (this->_active_drive.track / 2) * temp_r;
	// delta is now in meters/sec

	// determine if we will over-speed the motor and scal accordingly
	ratio = this->_convert_mps_to_proportional(temp_x + delta);
	if (ratio > 1.0)
		temp_x /= ratio;

	
	switch (this->_active_drive.mode) {

	case MODE_ACKERMAN:
		/*
		  with ackerman drive, steering is handled by a separate servo
		  we drive assigned servos exclusively by the linear.x
		*/
		speed[0] = temp_x * dir_x;
		speed[0] = this->_convert_mps_to_proportional(speed[0]);
		if (this->_abs(speed[0]) > 1.0)
			speed[0] = 1.0 * dir_x;
		
		RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"ackerman drive mode speed=%6.4f", speed[0]);
		break;

	case MODE_DIFFERENTIAL:
		/*
		  with differential drive, steering is handled by the relative speed of left and right servos
		  we drive assigned servos by mixing linear.x and angular.z
		  we compute the delta for left and right components
		  we use the sign of the angular velocity to determine which is the faster / slower
		*/

		/* the delta is the angular velocity * half the drive track */
		
		if (dir_r > 0) {	// turning right
			speed[0] = (temp_x + delta) * dir_x;
			speed[1] = (temp_x - delta) * dir_x;
		} else {		// turning left
			speed[0] = (temp_x - delta) * dir_x;
			speed[1] = (temp_x + delta) * dir_x;
		}

		RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"computed differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1]);

		/* if any of the results are greater that 1.0, we need to scale all the results down */
		range = this->_max (this->_abs(speed[0]), this->_abs(speed[1]));
		
		ratio = this->_convert_mps_to_proportional(range);
		if (ratio > 1.0) {
			speed[0] /= ratio;
			speed[1] /= ratio;
		}
		RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"adjusted differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1]);

		speed[0] = this->_convert_mps_to_proportional(speed[0]);
		speed[1] = this->_convert_mps_to_proportional(speed[1]);

		RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1]);
		break;

	case MODE_MECANUM:
		/*
		  with mecanum drive, steering is handled by the relative speed of left and right servos
		  with mecanum drive, lateral motion is handled by the rotation of front and rear servos
		  we drive assigned servos by mixing linear.x and angular.z  and linear.y
		*/

		if (dir_r > 0) {	// turning right
			speed[0] = speed[2] = (temp_x + delta) * dir_x;
			speed[1] = speed[3] = (temp_x - delta) * dir_x;
		} else {		// turning left
			speed[0] = speed[2] = (temp_x - delta) * dir_x;
			speed[1] = speed[3] = (temp_x + delta) * dir_x;
		}

		speed[0] += temp_y * dir_y;
		speed[3] += temp_y * dir_y;
		speed[1] -= temp_y * dir_y;
		speed[2] -= temp_y * dir_y;
		RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"computed mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3]);

		range = this->_max (this->_max (this->_max (this->_abs(speed[0]), this->_abs(speed[1])), this->_abs(speed[2])), this->_abs(speed[3]));
		ratio = this->_convert_mps_to_proportional(range);
		if (ratio > 1.0) {
			speed[0] /= ratio;
			speed[1] /= ratio;
			speed[2] /= ratio;
			speed[3] /= ratio;
		}
		RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"adjusted mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3]);

		speed[0] = this->_convert_mps_to_proportional(speed[0]);
		speed[1] = this->_convert_mps_to_proportional(speed[1]);
		speed[2] = this->_convert_mps_to_proportional(speed[2]);
		speed[3] = this->_convert_mps_to_proportional(speed[3]);

		RCLCPP_DEBUG(rclcpp::get_logger("i2cpwm_ros2"),"mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3]);
		break;

	default:
		break;

	}
	
	/* find all drive servos and set their new speed */
	for (i=0; i<(_last_servo); i++) {
		// we use 'fall thru' on the switch statement to allow all necessary servos to be controlled
		switch (this->_active_drive.mode) {
		case MODE_MECANUM:
			if (this->_servo_configs[i].mode_pos == POSITION_RIGHTREAR)
				this->_set_pwm_interval_proportional (i+1, speed[3]);
			if (this->_servo_configs[i].mode_pos == POSITION_LEFTREAR)
				this->_set_pwm_interval_proportional (i+1, speed[2]);
		case MODE_DIFFERENTIAL:
			if (this->_servo_configs[i].mode_pos == POSITION_RIGHTFRONT)
			    this->_set_pwm_interval_proportional (i+1, speed[1]);
		case MODE_ACKERMAN:
			if (this->_servo_configs[i].mode_pos == POSITION_LEFTFRONT)
				this->_set_pwm_interval_proportional (i+1, speed[0]);
		}
	}
}

void i2cpwm_ros2::set_pwm_frequency (const std::shared_ptr<i2cpwm_ros2_msgs::srv::IntValue::Request> req,
          std::shared_ptr<i2cpwm_ros2_msgs::srv::IntValue::Response> res)
{
	int freq;
	freq = req->value;
	if ((freq<12) || (freq>1024)) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Invalid PWM frequency %d :: PWM frequencies should be between 12 and 1024", freq);
		freq = 50;	// most analog RC servos are designed for 20ms pulses.
		res->error = freq;
	}
	this->_set_pwm_frequency (freq);	// I think we must reset frequency when we change boards
	res->error = freq;
}

void i2cpwm_ros2::config_servos (const std::shared_ptr<i2cpwm_ros2_msgs::srv::ServosConfig::Request> req,
          std::shared_ptr<i2cpwm_ros2_msgs::srv::ServosConfig::Response> res)
{
	/* this service works on the active_board */
	int i;
	
	res->error = 69;

	if ((this->_active_board<1) || (this->_active_board>62)) {
		RCLCPP_ERROR(rclcpp::get_logger("i2cpwm_ros2"),"Internal error - invalid board number %d :: PWM board numbers must be between 1 and 62", this->_active_board);
		res->error = -1;
	}

	for (i=0;i<req->servos.size();i++) {
		int servo = req->servos[i].servo;
		int center = req->servos[i].center;
		int range = req->servos[i].range;
		int direction = req->servos[i].direction;

		this->_config_servo (servo, center, range, direction);
		// not needed RCLCPP_INFO(rclcpp::get_logger("i2cpwm_ros2"),"Servo configured: #%d, Center: %d, Range: %d, Direction: %d",req->servos[i].servo, req->servos[i].center, req->servos[i].range, req->servos[i].direction);
	}

}

void i2cpwm_ros2::config_drive_mode (const std::shared_ptr<i2cpwm_ros2_msgs::srv::DriveMode::Request> req,
          std::shared_ptr<i2cpwm_ros2_msgs::srv::DriveMode::Response> res)
{
	res->error = 0;

	int i;

	if ((res->error = this->_config_drive_mode (req->mode, req->rpm, req->radius, req->track, req->scale)))
		return;

	for (i=0;i<req->servos.size();i++) {
		int servo = req->servos[i].servo;
		int position = req->servos[i].position;

		if (this->_config_servo_position (servo, position) != 0) {
			res->error = servo; /* this needs to be more specific and indicate a bad server ID was provided */
			continue;
		}
	}

}

void i2cpwm_ros2::stop_servos (const std::shared_ptr<std_srvs::srv::Empty::Request> req,
          std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
	int save_active = this->_active_board;
	int i;

	for (i=0; i<MAX_BOARDS; i++) {
		if (this->_pwm_boards[i] > 0) {
			this->_set_active_board (i+1);	// API is ONE based
			this->_set_pwm_interval_all (0, 0);
		}
	}
	this->_set_active_board (save_active);	// restore last active board
}

void i2cpwm_ros2::handle_logger_level()
{
	const char * severity_string = this->_logging_level_str.c_str();
  RCLCPP_INFO(
    rclcpp::get_logger("i2cpwm_ros2"), "Incoming Log Level request: severity '%s'",
    severity_string);
  std::flush(std::cout);

  int severity;
  rcutils_ret_t ret = rcutils_logging_severity_level_from_string(
    severity_string, rcl_get_default_allocator(), &severity);
  if (RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID == ret) {
    RCLCPP_ERROR(
      rclcpp::get_logger("i2cpwm_ros2"), "Unknown severity '%s'", severity_string);
    return;
  }
  if (RCUTILS_RET_OK != ret) {
    RCLCPP_ERROR(
      rclcpp::get_logger("i2cpwm_ros2"), "Error %d getting severity level from request: %s", ret,
      rcl_get_error_string().str);
    rcl_reset_error();
    return;
  }

  ret = rcutils_logging_set_logger_level("i2cpwm_ros2", severity);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger("i2cpwm_ros2"), "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

// Main Loop
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<i2cpwm_ros2>());
  rclcpp::shutdown();
  return 0;
}
