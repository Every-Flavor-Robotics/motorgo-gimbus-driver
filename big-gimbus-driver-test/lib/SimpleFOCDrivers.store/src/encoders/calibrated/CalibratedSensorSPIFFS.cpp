#include "CalibratedSensorSPIFFS.h"
#include "SPIFFS.h"

// CalibratedSensor()
// sensor              - instance of original sensor object
// n_lut               - number of samples in the LUT
CalibratedSensor::CalibratedSensor(Sensor &wrapped, int n_lut, float *lut)
    : _wrapped(wrapped), n_lut(n_lut) {
		this->calibrationLut = this->cal_data.lut;
	};

CalibratedSensor::~CalibratedSensor() {
};

// call update of calibrated sensor
void CalibratedSensor::update()
{
	_wrapped.update();
	this->Sensor::update();
};

// Retrieve the calibrated sensor angle
void CalibratedSensor::init()
{
	// assume wrapped sensor has already been initialized
	this->Sensor::init(); // call superclass init
}

// Retrieve the calibrated sensor angle
float CalibratedSensor::getSensorAngle()
{
	if(!calibrationLut) {
		return _wrapped.getMechanicalAngle();
	}
    // raw encoder position e.g. 0-2PI
    float raw_angle = fmodf(_wrapped.getMechanicalAngle(), _2PI);
    raw_angle += raw_angle < 0 ? _2PI:0;

    // Calculate the resolution of the LUT in radians
    float lut_resolution = _2PI / n_lut;
    // Calculate LUT index
    int lut_index = raw_angle / lut_resolution;

    // Get calibration values from the LUT
    float y0 = calibrationLut[lut_index];
    float y1 = calibrationLut[(lut_index + 1) % n_lut];

    // Linearly interpolate between the y0 and y1 values
    // Calculate the relative distance from the y0 (raw_angle has to be between y0 and y1)
    // If distance = 0, interpolated offset = y0
    // If distance = 1, interpolated offset = y1
    float distance = (raw_angle - lut_index * lut_resolution) / lut_resolution;
    float offset = (1 - distance) * y0 + distance * y1;

    // Calculate the calibrated angle
    return raw_angle - offset;
}

// Perform filtering to linearize position sensor eccentricity
// FIR n-sample average, where n = number of samples in the window
// This filter has zero gain at electrical frequency and all integer multiples
// So cogging effects should be completely filtered out
void CalibratedSensor::filter_error(float* error, float &error_mean, int n_ticks, int window){
	float window_buffer[window];
	memset(window_buffer, 0, window*sizeof(float));
	float window_sum = 0;
	int buffer_index = 0;
	// fill the inital window buffer
	for (int i = 0; i < window; i++) {
		int ind = n_ticks - window/2 -1 + i;
		window_buffer[i] = error[ind % n_ticks];
		window_sum += window_buffer[i];
	}
	// calculate the moving average
	error_mean = 0;
	for (int i = 0; i < n_ticks; i++)
	{
		// Update buffer
		window_sum -= window_buffer[buffer_index];
		window_buffer[buffer_index] = error[( i + window/2 ) %n_ticks];
		window_sum += window_buffer[buffer_index];
		// update the buffer index
		buffer_index = (buffer_index + 1) % window;

		// Update filtered error
		error[i] = window_sum / (float)window;
		// update the mean value
		error_mean += error[i] / n_ticks;
	}

}

void CalibratedSensor::calibrate(FOCMotor &motor, int settle_time_ms)
{
	motor.monitor_port->println("Starting Sensor Calibration.");
	// Calibration variables
	
    // Init inital angles
    float theta_actual = 0.0;
    float avg_elec_angle = 0.0;
	// set the inital electric angle to 0
    float elec_angle = 0.0;

	// Calibration parameters
	// The motor will take a n_pos samples per electrical cycle
	// which amounts to n_ticks (n_pos * motor.pole_pairs) samples per mechanical rotation
	// Additionally, the motor will take n2_ticks steps to reach any of the n_ticks posiitons
	// incrementing the electrical angle by deltaElectricalAngle each time
	int n_pos = 5;
	int _NPP = motor.pole_pairs;								      // number of pole pairs which is user input
	const int n_ticks = n_pos * _NPP;							      // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
	const int n2_ticks = 5;										      // increments between saved samples (for smoothing motion)
	float deltaElectricalAngle = _2PI * _NPP / (n_ticks * n2_ticks);  // Electrical Angle increments for calibration steps
	float *error = new float[n_ticks];	         						  		  // pointer to error array (average of forward & backward)
	memset(error, 0, n_ticks*sizeof(float));
	// The fileter window size is set to n_pos - one electrical cycle 
	// important for cogging filtering !!!
	const int window = n_pos; // window size for moving average filter of raw error
	

	// find the first guess of the motor.zero_electric_angle 
	// and the sensor direction
	// updates motor.zero_electric_angle
	// updates motor.sensor_direction
	// temporarily unlink the sensor and current sense
	CurrentSense *current_sense = motor.current_sense;
	motor.current_sense = nullptr;
	motor.linkSensor(&this->_wrapped);
	if(!motor.initFOC()){
		motor.monitor_port->println("Failed to align the sensor.");
		return;
	}
	// link back the sensor and current sense
	motor.linkSensor(this);
	motor.linkCurrentSense(current_sense);

	// Set voltage angle to zero, wait for rotor position to settle
	// keep the motor in position while getting the initial positions
	motor.setPhaseVoltage(1, 0, elec_angle);
	_delay(1000);
	_wrapped.update();
	float theta_init = _wrapped.getAngle();
	float theta_absolute_init = _wrapped.getMechanicalAngle();

	/*
	Start Calibration
	Loop over  electrical angles from 0 to NPP*2PI, once forward, once backward
	store actual position and error as compared to electrical angle
	*/

	/*
	forwards rotation
	*/
	motor.monitor_port->print("Rotating: ");
	motor.monitor_port->println( motor.sensor_direction == Direction::CCW ? "CCW" : "CW" );
	float zero_angle_prev = 0.0;
	for (int i = 0; i < n_ticks; i++)
	{
		for (int j = 0; j < n2_ticks; j++) // move to the next location
		{
			_wrapped.update();
			elec_angle += deltaElectricalAngle;
			motor.setPhaseVoltage(voltage_calibration, 0, elec_angle);
		}

		// delay to settle in position before taking a position sample
		_delay(settle_time_ms);
		_wrapped.update();
		// calculate the error
		theta_actual = (int)motor.sensor_direction * (_wrapped.getAngle() - theta_init);
        error[i] = 0.5 * (theta_actual - elec_angle / _NPP);

		// calculate the current electrical zero angle
		float zero_angle = ((int)motor.sensor_direction * _wrapped.getMechanicalAngle() * _NPP ) - (elec_angle + _PI_2);
		zero_angle = _normalizeAngle(zero_angle);
		// remove the 2PI jumps
		if(zero_angle - zero_angle_prev > _PI){
			zero_angle = zero_angle - _2PI;
		}else if(zero_angle - zero_angle_prev < -_PI){
			zero_angle = zero_angle + _2PI;
		}
		zero_angle_prev = zero_angle;
		avg_elec_angle += zero_angle/n_ticks;

		// motor.monitor_port->print(">zero:");
		// motor.monitor_port->println(zero_angle);
		// motor.monitor_port->print(">zero_average:");
		// motor.monitor_port->println(avg_elec_angle/2.0);


	}
	_delay(2000);

	/*
	backwards rotation
	*/
	motor.monitor_port->print("Rotating: ");
	motor.monitor_port->println( motor.sensor_direction == Direction::CCW ? "CW" : "CCW" );
	for (int i = n_ticks - 1; i >= 0; i--)
	{
		for (int j = 0; j < n2_ticks; j++) // move to the next location
		{
			_wrapped.update();
			elec_angle -= deltaElectricalAngle;
			motor.setPhaseVoltage(voltage_calibration, 0, elec_angle);
		}

		// delay to settle in position before taking a position sample
		_delay(settle_time_ms);
		_wrapped.update();
		// calculate the error
		theta_actual = (int)motor.sensor_direction * (_wrapped.getAngle() - theta_init);
        error[i] += 0.5 * (theta_actual - elec_angle / _NPP);
		// calculate the current electrical zero angle
		float zero_angle = ((int)motor.sensor_direction * _wrapped.getMechanicalAngle() * _NPP ) - (elec_angle + _PI_2);
		zero_angle = _normalizeAngle(zero_angle);
		// remove the 2PI jumps
		if(zero_angle - zero_angle_prev > _PI){
			zero_angle = zero_angle - _2PI;
		}else if(zero_angle - zero_angle_prev < -_PI){
			zero_angle = zero_angle + _2PI;
		}
		zero_angle_prev = zero_angle;
		avg_elec_angle += zero_angle/n_ticks;

		// motor.monitor_port->print(">zero:");
		// motor.monitor_port->println(zero_angle);
		// motor.monitor_port->print(">zero_average:");
		// motor.monitor_port->println(avg_elec_angle/2.0);
	}

	// get post calibration mechanical angle.
	_wrapped.update();
	float theta_absolute_post = _wrapped.getMechanicalAngle();

	// done with the measurement
	motor.setPhaseVoltage(0, 0, 0);

	// raw offset from initial position in absolute radians between 0-2PI
	float raw_offset = (theta_absolute_init + theta_absolute_post) / 2;

	// calculating the average zero electrical angle from the forward calibration.
	motor.zero_electric_angle = _normalizeAngle(avg_elec_angle / (2.0));
	motor.monitor_port->print("Average Zero Electrical Angle: ");
	motor.monitor_port->println(motor.zero_electric_angle);
	_delay(1000);

	// Perform filtering to linearize position sensor eccentricity
	// FIR n-sample average, where n = number of samples in one electrical cycle
	// This filter has zero gain at electrical frequency and all integer multiples
	// So cogging effects should be completely filtered out
	float error_mean = 0.0;
	this->filter_error(error, error_mean, n_ticks, window);

	_delay(1000);
	// calculate offset index
	int index_offset = floor((float)n_lut * raw_offset / _2PI);
	float dn = n_ticks / (float)n_lut;

	motor.monitor_port->println("Constructing LUT.");
	_delay(1000);
	// Build Look Up Table
	for (int i = 0; i < n_lut; i++)
	{
		int ind = index_offset + i*motor.sensor_direction;
		if (ind > (n_lut - 1)) ind -= n_lut;
		if (ind < 0) ind += n_lut;
        // UPDATED: Populate the lut within the struct
		cal_data.lut[ind] = (float)(error[(int)(i * dn)] - error_mean); 
		cal_data.lut[ind] =  (int)motor.sensor_direction * cal_data.lut[ind];
	}
	motor.monitor_port->println("");
	_delay(1000);

	// Display the LUT
	motor.monitor_port->print("float calibrationLut[");
	motor.monitor_port->print(n_lut);
	motor.monitor_port->println("] = {");
	_delay(100); 
	for (int i=0;i < n_lut; i++){
		motor.monitor_port->print(cal_data.lut[i],6);
		if(i < n_lut - 1) motor.monitor_port->print(", ");
		_delay(1);
	}
	motor.monitor_port->println("};");
	_delay(1000);

	// Display the zero electrical angle
	motor.monitor_port->print("float zero_electric_angle = ");
	motor.monitor_port->print(motor.zero_electric_angle,6);
	motor.monitor_port->println(";");

	// Display the sensor direction
	motor.monitor_port->print("Direction sensor_direction = ");
	motor.monitor_port->println(motor.sensor_direction == Direction::CCW ? "Direction::CCW;" : "Direction::CW;");
	_delay(1000);

    if(saveCalibration(motor)){
       Serial.println("Calibration data saved successfully.");
    } else {
       Serial.println("Failed to save calibration data.");
    }

	// Free error memory from heap
	delete[] error;
	motor.monitor_port->println("Sensor Calibration Done.");
}

// Add this new method to save the data
bool CalibratedSensor::saveCalibration(FOCMotor& motor){
    // Populate the struct with the final motor values
    cal_data.zero_electric_angle = motor.zero_electric_angle;
    cal_data.sensor_direction = (int)motor.sensor_direction;
    // The LUT is already in cal_data.lut

    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return false;
    }

    File file = SPIFFS.open("/calibration.bin", FILE_WRITE);
    if(!file){
        Serial.println("There was an error opening the file for writing");
        return false;
    }

    // Write the raw bytes of the entire struct to the file
    int bytesWritten = file.write((uint8_t*)&cal_data, sizeof(CalibrationData));
    
    file.close();

    return bytesWritten == sizeof(CalibrationData);
}


bool CalibratedSensor::loadCalibration(FOCMotor& motor){
    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return false;
    }

    if(!SPIFFS.exists("/calibration.bin")){
        Serial.println("Calibration file not found.");
        return false;
    }

    File file = SPIFFS.open("/calibration.bin", FILE_READ);
    if(!file){
        Serial.println("There was an error opening the file for reading");
        return false;
    }

    // Read the raw bytes from the file into our struct
    int bytesRead = file.read((uint8_t*)&cal_data, sizeof(CalibrationData));

    file.close();

    if(bytesRead == sizeof(CalibrationData)){
        Serial.println("Calibration data loaded successfully.");
        
        // IMPORTANT: Apply the loaded values to the motor object
        motor.zero_electric_angle = cal_data.zero_electric_angle;
        motor.sensor_direction = (Direction)cal_data.sensor_direction;
        
        // The LUT is already loaded into cal_data.lut, and our calibrationLut pointer is already set
        
        init(); 
        return true;
    } else {
        Serial.println("Failed to read correct number of bytes for calibration data.");
        return false;
    }
}