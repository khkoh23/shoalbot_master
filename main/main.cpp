#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

#include <std_msgs/msg/int32.h>
//#include <std_msgs/msg/int32_multi_array.h>
//#include <std_msgs/msg/multi_array_dimension.h>
//#include <std_msgs/msg/float32.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <nav_msgs/msg/odometry.h>
//#include <shoalbot_msgs/msg/fast.h>
//#include <shoalbot_msgs/msg/medium.h>
//#include <shoalbot_msgs/msg/slow.h>

#include "esp32_serial_transport.h"
#include "kinco_can.h"
#include "amip4k_spi.h"
#include "icm42688_spi.h"
#include "movingaverage_filter.h"
#include "bms_485.h"
#include "shoalbot_master_i2c.h"
#include "estop.h"

//#include "esp_intr_alloc.h"
//#include "esp_intr_types.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

//#define ROS_NAMESPACE CONFIG_MICRO_ROS_NAMESPACE

#define JETSON_22 GPIO_NUM_0
#define PWM_1 GPIO_NUM_1
#define PWM_2 GPIO_NUM_2
#define RS2_DE GPIO_NUM_3
#define CAN2_TX GPIO_NUM_4
#define CAN2_RX GPIO_NUM_5
#define CAN1_TX GPIO_NUM_6
#define CAN1_RX GPIO_NUM_7
#define RS1_DE GPIO_NUM_8
#define SPI_CS_L GPIO_NUM_9
#define SPI_CS_IMU GPIO_NUM_10
#define SPI_MOSI GPIO_NUM_11 
#define SPI_SCLK GPIO_NUM_12
#define SPI_MISO GPIO_NUM_13 
#define DO_17 GPIO_NUM_14
#define I2C_SDA GPIO_NUM_15
#define I2C_SCL GPIO_NUM_16
#define RS1_TX GPIO_NUM_17
#define RS1_RX GPIO_NUM_18
#define RS2_TX GPIO_NUM_19
#define RS2_RX GPIO_NUM_20
#define DO_18 GPIO_NUM_21
#define SPI_CS_R GPIO_NUM_35
#define DO_10 GPIO_NUM_36
#define DO_11 GPIO_NUM_37
#define DO_12 GPIO_NUM_38
#define DO_13 GPIO_NUM_39
#define DO_14 GPIO_NUM_40
#define DO_15 GPIO_NUM_41
#define DO_16 GPIO_NUM_42
#define DO_19 GPIO_NUM_47

unsigned long long time_offset = 0;
unsigned long prev_odom_update = 0;
unsigned long prev_time_update = 0;
float x_pos_ = 0.0;
float y_pos_ = 0.0;
float heading_ = 0.0;
const int interpolation_rate_ = 4;
const int encoder_resolution_ = 1024;
const float wheel_diameter_ = 0.15; 
const float wheel_base_ = 0.469; 
const int gear_ratio_ = 15;
int cpr = interpolation_rate_ * encoder_resolution_;
double gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z; 

int32_t left_speed, right_speed;
int16_t left_count_now, left_count_prev, right_count_now, right_count_prev;
int32_t left_counter, left_counter_prev, right_counter, right_counter_prev;
float left_speed_m, right_speed_m;

rcl_publisher_t imu_publisher, odom_publisher, bms_publisher, di_publisher; //, custom_publisher;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__BatteryState bms_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Int32 di_msg;
//my_custom_message__msg__MyCustomMessage custom_msg;
rcl_subscription_t twist_subscriber, do_subscriber;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int32 do_msg;

static size_t uart_port = UART_NUM_0; // UART port for micro-ROS

icm42688_spi_config imu_spi_config = {
	.miso = SPI_MISO, 
	.mosi = SPI_MOSI, 
	.sclk = SPI_SCLK, 
	.cs = SPI_CS_IMU, 
	.init_bus = 0
};
icm42688_spi imu(&imu_spi_config);

amip4k_spi_config left_spi_config = {
	.miso = SPI_MISO, 
	.mosi = SPI_MOSI, 
	.sclk = SPI_SCLK, 
	.cs = SPI_CS_L, 
	.hwa = 0b000, 
	.init_bus = 1
};
amip4k_spi left_encoder(&left_spi_config);

amip4k_spi_config right_spi_config = {
	.miso = SPI_MISO, 
	.mosi = SPI_MOSI, 
	.sclk = SPI_SCLK, 
	.cs = SPI_CS_R, 
	.hwa = 0b000, 
	.init_bus = 1
};
amip4k_spi right_encoder(&right_spi_config);

MovingAverageFilter left_speed_filter(1), right_speed_filter(1);
MovingAverageFilter left_input_filter(1), right_input_filter(1);

int32_t left_input_filtered, right_input_filtered;
float left_speed_filtered, right_speed_filtered;

i2c_master_config i2c_config = {
	.sda = I2C_SDA, 
	.scl = I2C_SCL, 
	.slaveAddr = 0x0A
};
shoalbot_master_i2c my_i2c(&i2c_config);
int32_t di_buffer;
bool new_DO = false;
bool new_DI = false;
uint8_t slave_do[3] = {0xBB, 0x00, 0x0B}; // first byte to indicating DO cmd, second bit 2 MSB
uint16_t master_do = 0;

bms_485 bms;
ESTOP estop;


void reset_gpio() {
    gpio_reset_pin(DO_10); // 1024
    gpio_reset_pin(DO_11); // 2048
    gpio_reset_pin(DO_12); // 4096
    gpio_reset_pin(DO_13); // 8192
    gpio_reset_pin(DO_14); // 16384
    gpio_reset_pin(DO_15); // 32768
    gpio_reset_pin(DO_16); // 65536
    gpio_reset_pin(DO_17); // 131072
    gpio_reset_pin(DO_18); // 262144
    gpio_reset_pin(DO_19); // 524288

    gpio_set_direction(DO_10, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_11, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_12, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_13, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_14, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_15, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_16, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_17, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_18, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_19, GPIO_MODE_OUTPUT);
}

void set_DO() {
	uint16_t master_do = 0;
    bool do10 = master_do & 0x0001;
    bool do11 = master_do & 0x0002;
    bool do12 = master_do & 0x0004;
    bool do13 = master_do & 0x0008;
    bool do14 = master_do & 0x0010;
    bool do15 = master_do & 0x0020;
    bool do16 = master_do & 0x0040;
    bool do17 = master_do & 0x0080;
    bool do18 = master_do & 0x0100;
    bool do19 = master_do & 0x0200;

    gpio_set_level(DO_10, do10);
    gpio_set_level(DO_11, do11);
    gpio_set_level(DO_12, do12);
    gpio_set_level(DO_13, do13);
    gpio_set_level(DO_14, do14);
    gpio_set_level(DO_15, do15);
    gpio_set_level(DO_16, do16);
    gpio_set_level(DO_17, do17);
    gpio_set_level(DO_18, do18);
    gpio_set_level(DO_19, do19);
}


void twist_callback(const void * msgin) {
	left_speed = (twist_msg.linear.x - (twist_msg.angular.z*wheel_base_*0.5)) / (wheel_diameter_*M_PI) * 60 * gear_ratio_; 
	right_speed = (twist_msg.linear.x + (twist_msg.angular.z*wheel_base_*0.5)) / (wheel_diameter_*M_PI) * 60 * gear_ratio_; 
	left_input_filtered = left_input_filter.process(left_speed);
	right_input_filtered = right_input_filter.process(right_speed);
}

void do_callback(const void * msgin) {
	uint32_t do_data = do_msg.data;
	slave_do[2] = do_data & 0x000000FF; // Do0-DO7 (Slave)
    slave_do[1] = (do_data & 0x00000300) >> 8; // Do8-DO9 (Slave)
    master_do = (do_data & 0x000FFC00) >> 10; // Do10-DO19 (Master)
    new_DO = true;
}

void battery_ros_init(void) { // Initializes the ROS topic information for Batery
	bms_msg.power_supply_technology = 4;
	bms_msg.present = 1;
//	bms_msg.location = micro_ros_string_utilities_set(bms_msg.location, "amr_1");
//	bms_msg.serial_number = micro_ros_string_utilities_set(bms_msg.serial_number, "");
//	bms_msg.header.frame_id = micro_ros_string_utilities_set(bms_msg.header.frame_id, "battery");
}


void imu_ros_init(void) { // Initializes the ROS topic information for IMU
	imu_msg.angular_velocity.x = 0;
	imu_msg.angular_velocity.y = 0;
	imu_msg.angular_velocity.z = 0;
	imu_msg.linear_acceleration.x = 0;
	imu_msg.linear_acceleration.y = 0;
	imu_msg.linear_acceleration.z = 0;
	imu_msg.orientation.x = 0;
	imu_msg.orientation.y = 0;
	imu_msg.orientation.z = 0;
	imu_msg.orientation.w = 1;
	imu_msg.angular_velocity_covariance[0] = 0.0002;
	imu_msg.angular_velocity_covariance[4] = 0.0002;
	imu_msg.angular_velocity_covariance[8] = 0.0002;
	imu_msg.linear_acceleration_covariance[0] = 0.0012;
	imu_msg.linear_acceleration_covariance[4] = 0.0012;
	imu_msg.linear_acceleration_covariance[8] = 0.0012;
	imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");
	imu_msg.orientation_covariance[0] = 0.00001;
	imu_msg.orientation_covariance[4] = 0.00001;
	imu_msg.orientation_covariance[8] = 0.00001;
}

void odom_ros_init(void) {
	odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
	odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");
	odom_msg.pose.pose.position.z = 0.0;
	odom_msg.pose.covariance[0] = 0.0001;
	odom_msg.pose.covariance[7] = 0.0001;
	odom_msg.pose.covariance[35] = 0.0001;
	odom_msg.twist.twist.linear.y = 0.0;
	odom_msg.twist.twist.linear.z = 0.0;
	odom_msg.twist.twist.angular.x = 0.0;
	odom_msg.twist.twist.angular.y = 0.0;
	odom_msg.twist.covariance[0] = 0.0001;
	odom_msg.twist.covariance[7] = 0.0001;
	odom_msg.twist.covariance[35] = 0.0001;
}

void odom_euler_to_quat(float roll, float pitch, float yaw, float *q) {
	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);
	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);
	q[0] = cy * cp * cr + sy * sp * sr;
	q[1] = cy * cp * sr - sy * sp * cr;
	q[2] = sy * cp * sr + cy * sp * cr;
	q[3] = sy * cp * cr - cy * sp * sr;
}

unsigned long get_millisecond(void) { // Get the number of seconds since boot
	return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

static void sync_time(void) { // Calculate the time difference between the microROS agent and the MCU
	unsigned long now = get_millisecond();
	RCSOFTCHECK(rmw_uros_sync_session(10));
	unsigned long long ros_time_ms = rmw_uros_epoch_millis();
	time_offset = ros_time_ms - now;    
}

struct timespec get_timespec(void) { // Get timestamp
	struct timespec tp = {};
	unsigned long long now = get_millisecond() + time_offset; // deviation of synchronous time
	tp.tv_sec = now / 1000;
	tp.tv_nsec = (now % 1000) * 1000000;
	return tp;
}

void odom_update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z) {
	float delta_heading = angular_vel_z * vel_dt; // radians
	float cos_h = cos(heading_);
	float sin_h = sin(heading_);
	float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; // m
	float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; // m
	// calculate current position of the robot
	x_pos_ += delta_x;
	y_pos_ += delta_y;
	heading_ += delta_heading;
	// calculate robot's heading in quaternion angle. ROS has a function to calculate yaw in quaternion angle
	float q[4];
	odom_euler_to_quat(0, 0, heading_, q);
	// robot's position in x,y, and z
	odom_msg.pose.pose.position.x = x_pos_;
	odom_msg.pose.pose.position.y = y_pos_;
	// robot's heading in quaternion
	odom_msg.pose.pose.orientation.x = (double)q[1];
	odom_msg.pose.pose.orientation.y = (double)q[2];
	odom_msg.pose.pose.orientation.z = (double)q[3];
	odom_msg.pose.pose.orientation.w = (double)q[0];
	// linear speed from encoders
	odom_msg.twist.twist.linear.x = linear_vel_x;
	odom_msg.twist.twist.linear.y = linear_vel_y;
	// angular speed from encoders
	odom_msg.twist.twist.angular.z = angular_vel_z;
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		struct timespec time_stamp = get_timespec();
		imu_msg.header.stamp.sec = time_stamp.tv_sec;
		imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
		//imu_msg.header.stamp.sec = rmw_uros_epoch_millis()/1000.0;
		//imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
		imu_msg.angular_velocity.x = gyro_x; 
		imu_msg.angular_velocity.y = gyro_y; 
		imu_msg.angular_velocity.z = gyro_z; 
		imu_msg.linear_acceleration.x = accel_x; 
		imu_msg.linear_acceleration.y = accel_y; 
		imu_msg.linear_acceleration.z = accel_z; 
		RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
	}
}

void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
/*		unsigned long now = get_millisecond();
		float vel_dt = (now - prev_odom_update) / 1000.0;
		prev_odom_update = now;
		left_speed_m = (left_counter - left_counter_prev) / vel_dt / cpr * M_PI * wheel_diameter_; // measure m/s
		right_speed_m = (right_counter - right_counter_prev) / vel_dt / cpr * M_PI * wheel_diameter_; // measure m/s
		left_speed_filtered = left_speed_filter.process(left_speed_m);
		right_speed_filtered = right_speed_filter.process(right_speed_m);
		float Vx = (right_speed_filtered + left_speed_filtered) * 0.5; //robot m/s
		float Vy = 0;
		float Wz = (right_speed_filtered - left_speed_filtered) / wheel_base_; // robot rad/s
		odom_update(vel_dt, Vx, Vy, Wz);
//		test_msg.data = right_speed_m;
		test_msg.data = right_speed_filtered;
		left_counter_prev = left_counter;
		right_counter_prev = right_counter; */
		struct timespec time_stamp = get_timespec();
		odom_msg.header.stamp.sec = time_stamp.tv_sec;
		odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
		//odom_msg.header.stamp.sec = rmw_uros_epoch_millis()/1000.0;
		//odom_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
		RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
	}
}

void bms_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		struct timespec time_stamp = get_timespec();
		bms_msg.header.stamp.sec = time_stamp.tv_sec;
		bms_msg.header.stamp.nanosec = time_stamp.tv_nsec;
		//bms_msg.header.stamp.sec = rmw_uros_epoch_millis()/1000.0;
		//bms_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
		RCSOFTCHECK(rcl_publish(&bms_publisher, &bms_msg, NULL));
	}
}

void di_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		di_msg.data = di_buffer; 
		RCSOFTCHECK(rcl_publish(&di_publisher, &di_msg, NULL));
	}
}
/*
void custom_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		struct timespec time_stamp = get_timespec();
		custom_msg.stamp.sec = time_stamp.tv_sec;
		custom_msg.stamp.nanosec = time_stamp.tv_nsec;
		RCSOFTCHECK(rcl_publish(&custom_publisher, &custom_msg, NULL));
	}
}
*/

void micro_ros_task(void * arg) {
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	//rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	//RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
	
	// Setup support structure
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	//RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	
	// Create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	//rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "shoalbot_master", "", &support));

	// create publisher
	//RCCHECK(rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
	RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
	RCCHECK(rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/data_raw"));
	RCCHECK(rclc_publisher_init_default(&bms_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "bms"));
	RCCHECK(rclc_publisher_init_default(&di_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "di"));
//	RCCHECK(rclc_publisher_init_default(&custom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, MyCustomMessage), "custom"));

	// Create subscriber
	RCCHECK(rclc_subscription_init_default(&twist_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"))
	RCCHECK(rclc_subscription_init_default(&do_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "do"))

	// Create timer
	//rcl_timer_t imu_timer, odom_timer, bms_timer, custom_timer;
	rcl_timer_t imu_timer = rcl_get_zero_initialized_timer(); 
	rcl_timer_t odom_timer = rcl_get_zero_initialized_timer();
	rcl_timer_t bms_timer = rcl_get_zero_initialized_timer();
	rcl_timer_t di_timer = rcl_get_zero_initialized_timer();
//	rcl_timer_t custom_timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(20), imu_timer_callback));
	RCCHECK(rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(50), odom_timer_callback));
	RCCHECK(rclc_timer_init_default(&bms_timer, &support, RCL_S_TO_NS(5), bms_timer_callback));
	RCCHECK(rclc_timer_init_default(&di_timer, &support, RCL_MS_TO_NS(500), di_timer_callback));
//	RCCHECK(rclc_timer_init_default(&custom_timer, &support, RCL_MS_TO_NS(1000), custom_timer_callback));

	// Create executor
	//rclc_executor_t executor;
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
	unsigned int rcl_wait_timeout = 1000; 
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subsriber to executor
	RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &bms_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &di_timer));
//	RCCHECK(rclc_executor_add_timer(&executor, &custom_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &do_subscriber, &do_msg, &do_callback, ON_NEW_DATA));

	sync_time();

	// Spin forever
	while(1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); //100
		usleep(1000); // 100000
	}

	// Free resources
	RCCHECK(rcl_subscription_fini(&twist_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&do_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
	RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
	RCCHECK(rcl_publisher_fini(&bms_publisher, &node));
	RCCHECK(rcl_publisher_fini(&di_publisher, &node));
//	RCCHECK(rcl_publisher_fini(&custom_publisher, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}

void twai_task(void *arg) { // Kinco motor task
	while (1) {
		setTargetVelocity(1, left_input_filtered); 
		setTargetVelocity(2, right_input_filtered); 
//		setTargetVelocity(1, 50); 
//		setTargetVelocity(2, 50); 

	}
	vTaskDelete(NULL);
}

void spi_task(void *arg) { // IMU and safety encoder data task
	while (1) {
		gyro_x = imu.get_gyro_x(); 
		gyro_y = imu.get_gyro_y(); 
		gyro_z = imu.get_gyro_z(); 
		accel_x = imu.get_accel_x(); 
		accel_y = imu.get_accel_y(); 
		accel_z = imu.get_accel_z(); 
		vTaskDelay(pdMS_TO_TICKS(2));

		left_count_now = left_encoder.readMVAL() * -1;
		right_count_now = right_encoder.readMVAL();

//		unsigned long now = get_millisecond();
//		float vel_dt = (now - prev_odom_update) / 1000.0;
		unsigned long now = esp_timer_get_time();
		float vel_dt = (now - prev_odom_update) / 1000000.0;
		prev_odom_update = now;

		/*	
		moving forward: -3 -2 -1 0 1 2 3
		reset forward: 2045 2046 2047 0 1 2 3
		moving backward: 3 2 1 0 -1 -2 -3
		reset backward: -2045 -2046 -2047 -2048 -1 -2 -3
		*/
		if (right_count_now > right_count_prev) { // moving forward or reset after moving backward
			if (abs(right_count_now - right_count_prev) > 1024) right_counter = right_counter - (2048 - right_count_now + right_count_prev); //reset backward
			else right_counter = right_counter + (right_count_now - right_count_prev); //moving forward
		}
		else if (right_count_now < right_count_prev) { //moving backward or reset after moving forward
			if (abs(right_count_now - right_count_prev) > 1024) right_counter = right_counter + (right_count_now - right_count_prev + 2048); //reset forward
			else right_counter = right_counter - (right_count_prev - right_count_now); //moving backward
		}
		else {} 
		right_count_prev = right_count_now;

		if (left_count_now > left_count_prev) { // moving forward or reset after moving backward
			if (abs(left_count_now - left_count_prev) > 1024) left_counter = left_counter - (2048 - left_count_now + left_count_prev); //reset backward
			else left_counter = left_counter + (left_count_now - left_count_prev); //moving forward
		}
		else if (left_count_now < left_count_prev) { //moving backward or reset after moving forward
			if (abs(left_count_now - left_count_prev) > 1024) left_counter = left_counter + (left_count_now - left_count_prev + 2048);  //reset forward
			else left_counter = left_counter - (left_count_prev - left_count_now); //moving backward
		}
		else {} 
		left_count_prev = left_count_now;

//		unsigned long now = get_millisecond();
//		float vel_dt = (now - prev_odom_update) / 1000.0;
//		prev_odom_update = now;
		left_speed_m = (left_counter - left_counter_prev) / vel_dt / cpr * M_PI * wheel_diameter_; // measure m/s
		right_speed_m = (right_counter - right_counter_prev) / vel_dt / cpr * M_PI * wheel_diameter_; // measure m/s
		if (left_counter==left_counter_prev) left_speed_m = 0;
		if (right_counter==right_counter_prev) right_speed_m = 0;
		left_counter_prev = left_counter;
		right_counter_prev = right_counter;
		left_speed_filtered = left_speed_filter.process(left_speed_m);
		right_speed_filtered = right_speed_filter.process(right_speed_m);
		float Vx = (right_speed_filtered + left_speed_filtered) / 2.0; //robot m/s
		float Vy = 0;
		float Wz = (right_speed_filtered - left_speed_filtered) / wheel_base_; // robot rad/s
		odom_update(vel_dt, Vx, Vy, Wz);
		vTaskDelay(pdMS_TO_TICKS(2));

	}
	vTaskDelete(NULL);
}

void rs485_task(void *arg) { // BMS task
	while (1) {
        bms.getBMSData();
        bms_msg.voltage = bms.getVoltage();
        bms_msg.temperature = bms.getTemperature();
        bms_msg.current = bms.getCurrent();
        bms_msg.charge = bms.getCharge();
        bms_msg.capacity = bms.getCapacity();
        bms_msg.design_capacity = bms.getCapacity();
        bms_msg.percentage = bms.getCharge() / bms.getCapacity();
        bms_msg.cell_temperature.size = 3;
        bms_msg.cell_temperature.data = bms.getCellTemperature();
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	vTaskDelete(NULL);
}

void i2c_task(void *arg) { // I2C master task
	//i2c.cntrl_BMSpass(0b001); // Pass2 0, Pass1 0, BMS 1
	// DO 9 8 7 6 5 4 3 2 1 0
	//    0 0 0 0 0 0 1 0 1 1
//	vTaskDelay(pdMS_TO_TICKS(100));
//	uint8_t slave_do[3] = {0xBB, 0x00, 0x0B}; //first byte to indicate DO cmd, second bit 2 MSB TODO: global variable?
	//uint8_t slave_do[3] = {0xBB, 0x00, 0x08}; 
//	my_i2c.i2c_send_DO(slave_do);
//	vTaskDelay(pdMS_TO_TICKS(100));
	while (1) {
		di_buffer = my_i2c.read_di();
		vTaskDelay(pdMS_TO_TICKS(100));
		my_i2c.i2c_send_DO(slave_do);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	vTaskDelete(NULL);
}

extern "C" void app_main(void) {
	vTaskDelay(pdMS_TO_TICKS(10));
	estop.begin();
	initTwai(CAN1_TX, CAN1_RX);
	vTaskDelay(pdMS_TO_TICKS(10));
	battery_ros_init();
	imu.begin();
	imu_ros_init();
	left_encoder.begin();
	right_encoder.begin();
	vTaskDelay(pdMS_TO_TICKS(10));
	odom_ros_init();
	left_encoder.reset_cnt();
	right_encoder.reset_cnt();
	left_count_now = 0; left_count_prev = 0; right_count_now = 0; right_count_prev = 0;
	left_counter = 0; left_counter_prev = 0; right_counter = 0; right_counter_prev = 0;
	left_speed_m = 0; right_speed_m = 0;
	x_pos_ = 0.0; y_pos_ = 0.0; heading_ = 0.0;
	vTaskDelay(pdMS_TO_TICKS(10));
	my_i2c.begin();

	esp_intr_dump(NULL);

	#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
		rmw_uros_set_custom_transport(true, (void *) &uart_port, esp32_serial_open, esp32_serial_close, esp32_serial_write, esp32_serial_read);
	#else
		#error micro-ROS transports misconfigured
	#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

	xTaskCreate(micro_ros_task, "micro_ros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
	//xTaskCreatePinnedToCore(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL, 0);
	xTaskCreate(spi_task, "spi_task", 16000, NULL, 5, NULL);
	//xTaskCreatePinnedToCore(spi_task, "spi_task", 16000, NULL, 5, NULL, 1);
	xTaskCreate(rs485_task, "rs485_task", 16000, NULL, 5, NULL);
	//xTaskCreatePinnedToCore(rs485_task, "rs485_task", 16000, NULL, 5, NULL, 1);
	xTaskCreate(twai_task, "twai_task", 16000, NULL, 5, NULL);
	//xTaskCreatePinnedToCore(twai_task, "twai_task", 16000, NULL, 5, NULL, 1);
	xTaskCreate(i2c_task, "i2c_task", 16000, NULL, 5, NULL);
	//xTaskCreatePinnedToCore(i2c_task, "i2c_task", 16000,  NULL, 5, NULL, 0);
}