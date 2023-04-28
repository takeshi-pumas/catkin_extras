//https://github.com/RobotJustina/JUSTINA/blob/develop/catkin_ws/src/planning/act_pln/src/carry_my_luggage.cpp
#include<iostream>
#include <sstream>
#include <algorithm>
#include <map>

#include "ros/ros.h"
#include "ros/time.h"

#include <cmath>
#include <vector> 
#include <string>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "robotino_msgs/DigitalReadings.h"
#include "actionlib_msgs/GoalStatus.h"


//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"

//Digital readings
#include "robotino_msgs/DigitalReadings.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#define GRAMMAR_POCKET_COMMANDS "grammars/GPSR.jsgf"

enum SMState {
	SM_INIT,
    SM_WAIT_FOR_DOOR,
    //SM_NAVIGATE_TO_INSTRUCTION_POINT,
    //SM_NAVIGATE_TO_LOCATION,
    SM_WAIT_FOR_INSTRUCTION,
    SM_FOLLOW_PERSON_GPSR,
    SM_FOLLOW_OPERATOR,
    SM_PICK_OBJECT,
    SM_FIND_PERSON_FOLLOW,
    SM_FIND_PERSON_GPSR,
    SM_RETURN_TO_INSTRUCTION_POINT,
    SM_FINISH_CHALLENGE,
    SM_FINISHED_TASK,
    SM_FIND_OBJECT_GPSR,
    SM_LOOK_FOR_PERSON,
};

    //Robotino Lights
    robotino_msgs::DigitalReadings arr_values;
    ros::Publisher pub_digital;

bool fail = false;
bool success = false;
SMState state = SM_INIT;
std::vector<float> goal_vec(3);
sensor_msgs::LaserScan laserScan;
    std::vector<std::string> findPersonDetect;

std::string lastRecoSpeech;
std::vector<std::string> tokens;

std::string grammarCommandsID = "GPSRCommands";
bool flag_door = true;

int tasks_finished = 0;

std::string object_person_location = "";
std::string place_to_take_the_object_to = "";
std::string object_person_name = "";

bool human_detector_bool = false;

void humanDetectorCallback(const std_msgs::Bool::ConstPtr& msg)
{
    human_detector_bool = msg -> data; 
}

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserScan = *msg;

    int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
    float laser_l=0;
    range=laserScan.ranges.size();
    std::cout<<laserScan.ranges.size()<<std::endl;
    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    //std::cout<<"Range Size: "<< range << "\n ";
    //std::cout<<"Range Central: "<< range_c << "\n ";
    //std::cout<<"Range Initial: "<< range_i << "\n ";
    //std::cout<<"Range Final: "<< range_f << "\n ";

    cont_laser=0;
    laser_l=0;
    for(int i=range_c-(range/10); i < range_c+(range/10); i++)
    {
        if(laserScan.ranges[i] > 0 && laserScan.ranges[i] < 4)
        { 
            laser_l=laser_l+laserScan.ranges[i]; 
            cont_laser++;
        }
    }
    //std::cout<<"Laser promedio: "<< laser_l/cont_laser << std::endl;    
    if(laser_l/cont_laser > 0.5)
    {
        flag_door = true;
        //std::cout<<"door open"<<std::endl;
    }
    else
    {
        flag_door = false;
        //std::cout<<"door closed"<<std::endl;
    }
}

//Locations
std::map<std::string, std::string> locations;//key = robot location, value = human location
void navigate_to_location(std::string location){
                std::cout << "Navigate to location" << std::endl;
                
                goal_vec = FestinoKnowledge::CoordenatesLocSrv(location);
                std::cout <<"Coordenates of " + locations[location] + ":"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000)){
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000)){
                    	std::cout << "Cannot move to " << locations[location] << std::endl;
                        FestinoHRI::say("Just let me go. Cries in robot",3);
                    }
                }
                FestinoHRI::say("I have arrived to " + locations[location],1);	
            	sleep(2);

                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
}

std::string find_name(std::vector<std::string> tokens){
    for(std::string name: tokens){
        if(name == "Jamie" || name == "Morgan" || name == "Michael" || name == "Jordan" || name == "Taylor" || name == "Tracy" ||
        name == "Robin" || name == "Alex" || name == "Coke" || name == "Apple" || name == "Mug" || name == "Soap" ||
        name == "Banana" || name == "Pitcher"){
            return name;
        }
    }

    return "not_found";
}

bool its_an_object(std::string name){
    if(name == "Coke" || name == "Apple" || name == "Mug" || name == "Soap" ||
        name == "Banana" || name == "Pitcher"){
            return true;
        }

        return false;
}

int main(int argc, char** argv){
	ros::Time::init();
	std::cout << "INITIALIZING GPSR NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;

    ros::Subscriber subLaserScan = n.subscribe("/scan", 1, callbackLaserScan);

    FestinoHRI::setNodeHandle(&n);
    FestinoVision::setNodeHandle(&n);
    FestinoNavigation::setNodeHandle(&n);
    FestinoKnowledge::setNodeHandle(&n);

    pub_digital = n.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000);
    ros::Subscriber sub_human = n.subscribe("human_detector_bool", 1000, humanDetectorCallback);

    ros::Rate loop(30);

    //Speaker
    std::string voice;

    //Locations
    locations["instruction_point"] = "instruction point";
    locations["exit"] = "exit";
    locations["entrance"] = "entrance";
    locations["kitchen"] = "kitchen";
    locations["dinning_room"] = "dinning room";
    locations["living_room"] = "living room";
    locations["bedroom"] = "bedroom";

    //bool human_detector_bool;

    //Robotino Lights
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};
    //This delay is necessary ... why?
    FestinoHRI::say(" ",3);

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
            {
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	    		FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);
	    		
	    		//White light
	    		arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                
	            ros::Duration(2, 0).sleep();
	            FestinoHRI::enableSpeechRecognized(false);
	            voice = "I am ready for the GPSR test";
	            FestinoHRI::say(voice, 3);
                
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
	    		
                state = SM_WAIT_FOR_DOOR;

                break;
            }
            case SM_WAIT_FOR_DOOR:
            {
                std::cout << "State machine: SM_WAIT_FOR_DOOR" << std::endl;
                FestinoHRI::say("I am waiting for the door to be open",3);
                //flag_door = false;
                if(flag_door){

                    arr_values.values = {0,0,0,0,1,1};
                    pub_digital.publish(arr_values);
                    ros::Duration(0.5, 0).sleep();
                    arr_values.values = {0,0,0,0,0,0};
                    pub_digital.publish(arr_values);
                    ros::Duration(0.5, 0).sleep();
                    arr_values.values = {0,0,0,0,1,1};
                    pub_digital.publish(arr_values);

                    FestinoHRI::say("I can see that the door is open, I am going to the instruction point",3);
                    sleep(3);

                    navigate_to_location("instruction_point");

                    state = SM_WAIT_FOR_INSTRUCTION;
                }
                break;

            }
            case SM_WAIT_FOR_INSTRUCTION:
            {
                std::cout << "State machine: SM_WAIT_FOR_INSTRUCTION" << std::endl;

                FestinoHRI::enableSpeechRecognized(true);
                
                FestinoHRI::say("I'm listening for instructions",3);
                    sleep(3);

/*
Go to {location}
Go to {location} and find {object}
Go to {location} and find {person}
Go to {location} and bring me {object}
*/

                tokens.clear();
                lastRecoSpeech = FestinoHRI::lastRecogSpeech();

                FestinoHRI::enableSpeechRecognized(false);

                if(lastRecoSpeech != ""){

                    tokens.clear();
                    boost::algorithm::split(tokens,lastRecoSpeech, boost::algorithm::is_any_of(" "));

                    if(tokens[0] == "Go" && tokens[1] == "to"){

                            for(std::string location : tokens){
                                if(location == "living"){
                                    object_person_location = "living_room";
                                    break;
                                }
                                if(location == "dinning"){
                                    object_person_location = "dinning_room";
                                    break;
                                }
                                if(location == "bedroom"){
                                    object_person_location = "bedroom";
                                    break;
                                }
                                if(location == "kitchen"){
                                    object_person_location = "kitchen";
                                    break;
                                }
                            }

                            bool go_to = true;
                            for(std::string tok : tokens){
                                if(tok == "find"){
                                    go_to = false;
                                    object_person_name = find_name(tokens);
                                    if(object_person_name != "not_found"){
                                        if(its_an_object(object_person_name)){//if its an object
                                            state = SM_FIND_OBJECT_GPSR;
                                        } else {
                                            state = SM_FIND_PERSON_GPSR;
                                        }
                                    } else {
                                        FestinoHRI::say("I didn't understood, could you please repeat the instruction?",3);	
                                        sleep(3);
                                    }
                                    break;
                                }
                                if(tok == "bring"){
                                    go_to = false;
                                    object_person_name = find_name(tokens);
                                    if(object_person_name != "not_found"){
                                        state = SM_PICK_OBJECT;
                                    } else {
                                        FestinoHRI::say("I didn't understood, could you please repeat the instruction?",3);	
                                        sleep(3);
                                    }
                                    break;
                                }
                            }
                            if(go_to){
                                navigate_to_location(object_person_location);
                                state = SM_FINISHED_TASK;
                                break;
                            }

                    } else {
                        FestinoHRI::say("I didn't understood, could you please repeat the instruction?",3);	
                        sleep(3);
                    }

                }

                break;
            }
            case SM_FOLLOW_PERSON_GPSR:
            {
                state = SM_FIND_PERSON_FOLLOW;
                break;
            }
            case SM_FIND_PERSON_FOLLOW:
            {

                std::cout << "State machine: SM_FOLLOW_PERSON" << std::endl;

    			if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Not found" << std::endl;

	    			voice = "I can't found you, please stand in front of me";
					FestinoHRI::say(voice, 5);

	    			FestinoHRI::enableHumanFollower(false);
	    		}
    			else{

    				if(human_detector_bool){
    					voice = "Say follow me when you are ready";
						FestinoHRI::say(voice, 2);

						//Waiting for command (red light)
						arr_values.values = {0,0,0,1,0,0};
	                	pub_digital.publish(arr_values);
	                	ros::Duration(0.5, 0).sleep();

						//Enable speech recognition
						FestinoHRI::enableSpeechRecognized(true);
						ros::Duration(2, 0).sleep();

						//Waiting for the operator to confirm
		    			if(FestinoHRI::waitForSpecificSentence("follow me", 5000)){
                            FestinoHRI::enableSpeechRecognized(false);
		    				
                            //Command recognized (green light)
							arr_values.values = {0,0,0,0,1,0};
			                pub_digital.publish(arr_values);
			                ros::Duration(0.5, 0).sleep();

		    				
		    				voice = "I'm going to follow you, please say here is the car if we reached the final destination";
							FestinoHRI::say(voice, 5);
		    				state = SM_FOLLOW_OPERATOR;
		    			}
    				}
    				else 
    				{
		    			voice = "I can't found you, please stand in front of me";
						FestinoHRI::say(voice, 5);
    				}
    				
	    		}
                break;
            }
	    	case SM_FOLLOW_OPERATOR:
            {
	    		std::cout << "State machine: SM_FOLLOW_OPERATOR" << std::endl;	
	    		
	    		//Following (turquoise light)
				arr_values.values = {0,0,0,0,1,1};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
/* Confirm Arrive location
	    		if(confirm_car_again){
	    			voice = "Please say here is the car if we reached the final destination";
	    			FestinoHRI::say(voice, 5);
	    			confirm_car_again = false;
	    		}*/
	    		
				FestinoHRI::enableHumanFollower(true);

	    		FestinoHRI::enableSpeechRecognized(true);

	    		if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Lost operator" << std::endl;
	    			voice = "I lost you";
					FestinoHRI::say(voice, 5);
	    			FestinoHRI::enableHumanFollower(false);
	    			state = SM_FIND_PERSON_FOLLOW;
                    break;
	    		}
    			/*
                Wait to arrive
    			if(FestinoHRI::waitForSpecificSentence("here is the car", 5000)){

    				//Command recognized (green light)
					arr_values.values = {0,0,0,0,1,0};
	                pub_digital.publish(arr_values);
	                ros::Duration(0.5, 0).sleep();

    				FestinoHRI::enableSpeechRecognized(false);
    				state = SM_WAIT_CONF_CAR;  SM_FINISHED_TASK
    			}
    			*/
	    		break;
            }

            case SM_PICK_OBJECT:
            {
                //Find the object
                navigate_to_location(object_person_location);

                //Pick up object

                    FestinoHRI::say("Please put the " + object_person_name + " in the tray, tell me, robot yes, when you put the " + object_person_name + " in the tray",5);
                    sleep(5);

				//Waiting for command (red light)
				arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();

				//Enable speech recognition
				FestinoHRI::enableSpeechRecognized(true);
				ros::Duration(2, 0).sleep();

				//Waiting for the operator to confirm
	    		if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){

	    			FestinoHRI::enableSpeechRecognized(false);

	    			//Command recognized (green light)
					arr_values.values = {0,0,0,0,1,0};
	                pub_digital.publish(arr_values);
	                ros::Duration(0.5, 0).sleep();

                    navigate_to_location(place_to_take_the_object_to);

                    FestinoHRI::say("Here is the " + object_person_name + ", plase take the " + object_person_name + " from the tray and tell me robot yes when you did it",5);
                    sleep(5);

	    		    if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
					    state = SM_FINISHED_TASK;
                    }
	    		}

                break;
            }
            case SM_FIND_PERSON_GPSR:
            {

                //Find the person
                navigate_to_location(object_person_location);

                state = SM_LOOK_FOR_PERSON;

                break;
            }
            case SM_LOOK_FOR_PERSON:
            {
                //Recognize 
                findPersonDetect = FestinoVision::enableRecogFacesName(true);
                for(std::string person_found : findPersonDetect){
                    if(person_found == object_person_name){
                        FestinoHRI::say(object_person_name + " I found you",3);
                        sleep(3);
                        state = SM_FINISHED_TASK;
                        break;
                    }
                }

                FestinoNavigation::moveDistAngle(0, 0.1, 100);
                break;
            }
            case SM_FIND_OBJECT_GPSR:
            {
                navigate_to_location(object_person_location);
                //Find object
                        FestinoHRI::say(" I found you the " + object_person_name,3);
                        sleep(3);
                        state = SM_FINISHED_TASK;
                
            }
            case SM_FINISHED_TASK:
            {
                std::cout << "State machine: SM_FINISH_CHALLENGE" << std::endl;

                tasks_finished++;
                if(tasks_finished >= 3){
                    state = SM_FINISH_CHALLENGE;
                } else {
                    state = SM_RETURN_TO_INSTRUCTION_POINT;
                }

                break;
            }
            case SM_RETURN_TO_INSTRUCTION_POINT:
            {

                    std::cout << "State machine: SM_RETURN_TO_INSTRUCTION_POINT" << std::endl;
    
                    arr_values.values = {0,0,0,0,1,1};
                    pub_digital.publish(arr_values);
                    ros::Duration(0.5, 0).sleep();
                    arr_values.values = {0,0,0,0,0,0};
                    pub_digital.publish(arr_values);
                    ros::Duration(0.5, 0).sleep();
                    arr_values.values = {0,0,0,0,1,1};
                    pub_digital.publish(arr_values);

                    FestinoHRI::say("I've completed the task number" + std::to_string(tasks_finished) + "I am going back to the instruction point for the next task",3);
                    sleep(3);

                    navigate_to_location("instruction_point");
                    state = SM_WAIT_FOR_INSTRUCTION;

                    break;
            }
            case SM_FINISH_CHALLENGE:
            {
                std::cout << "State machine: SM_FINISH_CHALLENGE" << std::endl;


                FestinoHRI::say("I've completed the GPSR challenge, bye",3);
                sleep(3);

                navigate_to_location("exit");
                
                success = true;
	    		fail = true;

                break;
            }

        }
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}