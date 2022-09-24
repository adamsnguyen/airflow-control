
/*******************************************************************************
 * Copyright (c) 2013-2020 Frank Pagliughi <fpagliughi@mindspring.com>
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Frank Pagliughi - initial implementation and documentation
 *******************************************************************************/

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "mqtt_helper.h"
#include "mqtt_sub_ros_pub.h"

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("paho_cpp_async_subcribe");
const std::string TOPIC("test");

#define QOS 1
#define N_RETRY_ATTEMPTS 5
#define DEBUG 1

/////////////////////////////////////////////////////////////////////////////

// Callbacks for the success or failures of requested actions.
// This could be used to initiate further action, but here we just log the
// results to the console.


void mqttHelper::action_listener::on_failure(const mqtt::token &tok)
{
	std::cout << name_ << " failure";
	if (tok.get_message_id() != 0)
		std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
	std::cout << std::endl;
}

void mqttHelper::action_listener::on_success(const mqtt::token &tok)
{
	std::cout << name_ << " success";
	if (tok.get_message_id() != 0)
		std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
	auto top = tok.get_topics();
	if (top && !top->empty())
		std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
	std::cout << std::endl;
}

mqttHelper::action_listener::action_listener(const std::string &name) : name_(name) {}

/////////////////////////////////////////////////////////////////////////////

/**
 * Local callback & listener class for use with the client connection.
 * This is primarily intended to receive messages, but it will also monitor
 * the connection to the broker. If the connection is lost, it will attempt
 * to restore the connection and re-subscribe to the topic.
 */
void mqttHelper::callback::reconnect()
{
	std::this_thread::sleep_for(std::chrono::milliseconds(2500));
	try
	{
		this->cli_.connect(connOpts_, nullptr, *this);
	}
	catch (const mqtt::exception &exc)
	{
		std::cerr << "Error: " << exc.what() << std::endl;
		exit(1);
	}
}

// Re-connection failure
void mqttHelper::callback::on_failure(const mqtt::token &tok)
{
	std::cout << "Connection attempt failed" << std::endl;
	if (++nretry_ > N_RETRY_ATTEMPTS)
		exit(1);
	reconnect();
}

// (Re)connection success
// Either this or connected() can be used for callbacks.
void mqttHelper::callback::on_success(const mqtt::token &tok) {}

// (Re)connection success
void mqttHelper::callback::connected(const std::string &cause)
{
	// std::cout << "\nConnection success" << std::endl;
	// std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
	// 	<< "\tfor client " << CLIENT_ID
	// 	<< " using QoS" << QOS << "\n"
	// 	<< "\nPress Q<Enter> to quit\n" << std::endl;

	for (auto &topic : this->mqttInterface_node->TOPICS) // access by reference to avoid copying
	{
		this->cli_.subscribe(topic, QOS, nullptr, subListener_);
	}
}

// Callback for when the connection is lost.
// This will initiate the attempt to manually reconnect.
void mqttHelper::callback::connection_lost(const std::string &cause)
{
	std::cout << "\nConnection lost" << std::endl;
	if (!cause.empty())
		std::cout << "\tcause: " << cause << std::endl;

	std::cout << "Reconnecting..." << std::endl;
	nretry_ = 0;
	reconnect();
}

// Callback for when a message arrives.
void mqttHelper::callback::message_arrived(mqtt::const_message_ptr msg)
{
	std::cout << "Message arrived" << std::endl;
	std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
	std::cout << "\tpayload: '" << msg->to_string() << "'\n"
			  << std::endl;
	
	auto messageFloat = std_msgs::msg::Float32();

	messageFloat.data = std::stof(msg->to_string());

	//Find the topic in the TOPICS list
	auto v = this->mqttInterface_node->TOPICS;
	int index;
	auto it = find(v.begin(), v.end(), msg->get_topic());

	// If topic was found
    if (it != v.end()) 
    {    
        // calculating the index
        index = it - v.begin();
    }
	else
	{
      std::stringstream output_message;
	  output_message.str(std::string());
      output_message << "Can't find topic: " << msg->get_topic();
      RCLCPP_INFO(this->mqttInterface_node->get_logger(), output_message.str());
	}

	this->mqttInterface_node->floatPublishers[index]->publish(messageFloat);

	if (DEBUG)
	{
		std::stringstream output_message;
	  	output_message.str(std::string());
      	output_message << "message topic: " << msg->get_topic() << " message data: " << messageFloat.data << " TOPIC index: " << index
		<< " TOPIC array element: " << this->mqttInterface_node->TOPICS[index];
      	RCLCPP_INFO(this->mqttInterface_node->get_logger(), output_message.str());
	}

}

void mqttHelper::callback::delivery_complete(mqtt::delivery_token_ptr token) {}

mqttHelper::callback::callback(mqtt::async_client &cli, mqtt::connect_options &connOpts, std::shared_ptr<MqttInterfaceRosPub> mqttInterface_node)
: nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription"), mqttInterface_node(mqttInterface_node)
{}
