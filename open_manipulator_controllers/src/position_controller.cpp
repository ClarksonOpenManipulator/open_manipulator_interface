#include "open_manipulator_controllers/position_controller.h"

PositionController::PositionController() : node_handle(""), priv_node_handle("~"), has_joint_state(false) {
	read_period = priv_node_handle.param<double>("dxl_read_period", 0.010f);
	write_period = priv_node_handle.param<double>("dxl_write_period", 0.010f);
	publish_period = priv_node_handle.param<double>("publish_period", 0.010f);
	dxl_wb = new DynamixelWorkbench;
}

PositionController::~PositionController() { }

bool PositionController::initWorkbench(const std::string port_name, const uint32_t baud_rate) {
	bool result = false;
	const char* log;

	result = dxl_wb->init(port_name.c_str(), baud_rate, &log);
	if (result == false) {
		ROS_ERROR("%s", log);
	}
	return result;
}

bool PositionController::getDynamixelsInfo(const std::string yaml_file) {
	YAML::Node dxl_node;
	dxl_node = YAML::LoadFile(yaml_file.c_str());

	if (dxl_node == NULL) return false;

	for (YAML::const_iterator it_file = dxl_node.begin(); it_file != dxl_node.end(); it_file++) {
		std::string name = it_file->first.as<std::string>();
		if (name.size() == 0) continue;

		YAML::Node item = dxl_node[name];
		for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++) {
			std::string item_name = it_item->first.as<std::string>();
			int32_t value = it_item->second.as<int32_t>();

			if (item_name == "ID") dynamixel[name] = value;

			ItemValue item_value = {item_name, value};
			std::pair<std::string, ItemValue> info(name, item_value);

			dynamixel_info.push_back(info);
		}
	}
	return true;
}

bool PositionController::loadDynamixels() {
	bool result = false;
	const char* log;

	for (std::pair<std::string, uint32_t> const& dxl : dynamixel) {
		uint16_t model_number = 0;
		result = dxl_wb->ping((uint8_t)dxl.second, &model_number, &log);
    	if (result == false) {
			ROS_ERROR("%s", log);
			ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
			return result;
		} else ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
	}
	return result;
}

bool PositionController::initDynamixels() {
	ROS_INFO("Torque Enabled");
	
	const char* log;

	for (std::pair<std::string, uint32_t> const& dxl : dynamixel) {
		dxl_wb->torqueOff((uint8_t)dxl.second);

		for (std::pair<std::string, ItemValue> const& info : dynamixel_info) {
			if (dxl.first == info.first) {
				if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate") {
					bool result = dxl_wb->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
					if (result == false) {
						ROS_ERROR("%s", log);
						ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
						return false;
					}
				}
			}
		}
		dxl_wb->torqueOn((uint8_t)dxl.second);
	}
	return true;
}

bool PositionController::initControlItems() {
	bool result = false;
	const char* log = NULL;

	uint32_t dxl_num = dynamixel.begin()->second;

	const ControlItem* goal_position = dxl_wb->getItemInfo(dxl_num, "Goal_Position");
	if (goal_position == NULL) return false;

	const ControlItem* present_position = dxl_wb->getItemInfo(dxl_num, "Present_Position");
	if (present_position == NULL) return false;

	const ControlItem* present_velocity = dxl_wb->getItemInfo(dxl_num, "Present_Velocity");
	if (present_velocity == NULL) return false;

	const ControlItem* present_current = dxl_wb->getItemInfo(dxl_num, "Present_Current");
	if (present_current == NULL) return false;

	control_items["Goal_Position"] = goal_position;

	control_items["Present_Position"] = present_position;
	control_items["Present_Velocity"] = present_velocity;
	control_items["Present_Current"] = present_current;

	return true;
}

bool PositionController::initSDKHandlers() {
	bool result = false;
	const char* log = NULL;
	
	result = dxl_wb->addSyncWriteHandler(control_items["Goal_Position"]->address, control_items["Goal_Position"]->data_length, &log);
	if (result == false) {
		ROS_ERROR("%s", log);
		return result;
	} else ROS_INFO("%s", log);
	
	uint16_t start_address = std::min(control_items["Present_Position"]->address, control_items["Present_Current"]->address);
    uint16_t read_length = control_items["Present_Position"]->data_length + 
						   control_items["Present_Velocity"]->data_length + 
						   control_items["Present_Current"]->data_length;

    result = dxl_wb->addSyncReadHandler(start_address, read_length, &log);
    if (result == false) {
		ROS_ERROR("%s", log);
		return result;
	}
	return result;
}

void PositionController::initPublisher() {
	dynamixel_state_list_pub = priv_node_handle.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
	joint_states_pub = priv_node_handle.advertise<sensor_msgs::JointState>("joint_states", 100);
}

void PositionController::initSubscriber() {
	goal_joint_state_sub = priv_node_handle.subscribe("position_command", 100, &PositionController::onJointStateGoal, this);
}

void PositionController::initServer() {
	dynamixel_command_server = priv_node_handle.advertiseService("dynamixel_command", &PositionController::dynamixelCommandMsgCallback, this);
}

void PositionController::onJointStateGoal(const sensor_msgs::JointState& msg) {
	goal_state = msg;
	has_joint_state = true;
}

void PositionController::readCallback(const ros::TimerEvent&) {
	bool result = false;
	const char* log = NULL;

	dynamixel_workbench_msgs::DynamixelState  dynamixel_state[dynamixel.size()];
	dynamixel_state_list.dynamixel_state.clear();

	int32_t get_current[dynamixel.size()];
	int32_t get_velocity[dynamixel.size()];
	int32_t get_position[dynamixel.size()];

	uint8_t id_array[dynamixel.size()];
	uint8_t id_cnt = 0;

	for (std::pair<std::string, uint32_t> const& dxl : dynamixel) {
		dynamixel_state[id_cnt].name = dxl.first;
		dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

		id_array[id_cnt++] = (uint8_t)dxl.second;
	}
	
	result = dxl_wb->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, dynamixel.size(), &log);
    if (result == false) {
		ROS_ERROR("%s", log);
	}

	result = dxl_wb->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, id_cnt,
									  control_items["Present_Current"]->address,
                                      control_items["Present_Current"]->data_length,
                                      get_current,
                                      &log);
	if (result == false) {
		ROS_ERROR("%s", log);
	}

	result = dxl_wb->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, id_cnt,
									  control_items["Present_Velocity"]->address,
									  control_items["Present_Velocity"]->data_length,
								      get_velocity,
									  &log);
	if (result == false) {
		ROS_ERROR("%s", log);
	}

	result = dxl_wb->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, id_array, id_cnt,
                                      control_items["Present_Position"]->address,
                                      control_items["Present_Position"]->data_length,
                                      get_position,
                                      &log);
	if (result == false) {
		ROS_ERROR("%s", log);
	}

	for(uint8_t index = 0; index < id_cnt; index++) {
		dynamixel_state[index].present_current = get_current[index];
		dynamixel_state[index].present_velocity = get_velocity[index];
        dynamixel_state[index].present_position = get_position[index];

        dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
	}	
}

void PositionController::writeCallback(const ros::TimerEvent& t) {
	if (has_joint_state == false) return;
	
	bool result = false;
	const char* log = NULL;
	
	uint8_t id_array[dynamixel.size()];
	uint8_t id_cnt = 0;

	int32_t dynamixel_position[dynamixel.size()];

	for (std::string name : goal_state.name) {
		id_array[id_cnt] = (uint8_t) dynamixel[name];
		id_cnt++;
	}
	
	for (uint8_t index = 0; index < id_cnt; index++) 
		dynamixel_position[index] = dxl_wb->convertRadian2Value(id_array[index], goal_state.position[index]);
	
	result = dxl_wb->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, dynamixel_position, 1, &log);
	if (result == false) ROS_ERROR("%s", log);
	
	has_joint_state = false;
}

void PositionController::publishCallback(const ros::TimerEvent&) {
	dynamixel_state_list_pub.publish(dynamixel_state_list);
	
	joint_state_msg.header.stamp = ros::Time::now();

    joint_state_msg.name.clear();
    joint_state_msg.position.clear();
    joint_state_msg.velocity.clear();
    joint_state_msg.effort.clear();

    uint8_t id_cnt = 0;
	
	for (std::pair<std::string, uint32_t> const& dxl : dynamixel) {
		double position = 0.0;
		double velocity = 0.0;
		double effort = 0.0;
      
		joint_state_msg.name.push_back(dxl.first);
		
		effort = dxl_wb->convertValue2Current((int16_t)dynamixel_state_list.dynamixel_state[id_cnt].present_current);
		velocity = dxl_wb->convertValue2Velocity((uint8_t)dxl.second, (int32_t)dynamixel_state_list.dynamixel_state[id_cnt].present_velocity);
		position = dxl_wb->convertValue2Radian((uint8_t)dxl.second, (int32_t)dynamixel_state_list.dynamixel_state[id_cnt].present_position);

		joint_state_msg.effort.push_back(effort);
		joint_state_msg.velocity.push_back(velocity);
		joint_state_msg.position.push_back(position);

		id_cnt++;
	}
	joint_states_pub.publish(joint_state_msg);
}

bool PositionController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req, dynamixel_workbench_msgs::DynamixelCommand::Response &res) {
	bool result = false;
	const char* log;

	uint8_t id = req.id;
	std::string item_name = req.addr_name;
	int32_t value = req.value;

	result = dxl_wb->itemWrite(id, item_name.c_str(), value, &log);
	if (result == false) {
		ROS_ERROR("%s", log);
		ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
	}

	res.comm_result = result;

	return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "position_controller");
	ros::NodeHandle node_handle("");
	
	std::string port_name = "/dev/ttyUSB0";
	uint32_t baud_rate = 57600;

	if (argc < 2) {
		ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
		return 0;
	} else {
		port_name = argv[1];
	    baud_rate = atoi(argv[2]);
	}
	
	PositionController position_controller;
	
	bool result = false;
	
	std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

	result = position_controller.initWorkbench(port_name, baud_rate);
	if (result == false) {
		ROS_ERROR("Please check USB port name");
		return 0;
	}

	result = position_controller.getDynamixelsInfo(yaml_file);
	if (result == false) {
 	   ROS_ERROR("Please check YAML file");
 	   return 0;
	}

	result = position_controller.loadDynamixels();
	if (result == false) {
		ROS_ERROR("Please check Dynamixel ID or BaudRate");
		return 0;
	}

	result = position_controller.initDynamixels();
	if (result == false) {
		ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
		return 0;
	}

	result = position_controller.initControlItems();
	if (result == false) {
		ROS_ERROR("Please check control items");
		return 0;
	}

	result = position_controller.initSDKHandlers();
	if (result == false) {
		ROS_ERROR("Failed to set Dynamixel SDK Handler");
		return 0;
	}
	
	position_controller.initPublisher();
	position_controller.initSubscriber();
	position_controller.initServer();
	
	ros::Timer read_timer = node_handle.createTimer(ros::Duration(position_controller.getReadPeriod()), &PositionController::readCallback, &position_controller);
	ros::Timer write_timer = node_handle.createTimer(ros::Duration(position_controller.getWritePeriod()), &PositionController::writeCallback, &position_controller);
	ros::Timer publish_timer = node_handle.createTimer(ros::Duration(position_controller.getPublishPeriod()), &PositionController::publishCallback, &position_controller);

	ros::spin();

	return 0;
}	
