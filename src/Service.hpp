#ifndef _SERVICE
#define _SERVICE

#include "ros/ros.h"
#include "beginner_tutorials/CamToAlg.h"


class Service
{
public :
	template <class T, class U>
	Service(const std::string& serviceName, bool (&updateFunction)(T&, U&)) :
		m_service(m_node.advertiseService(serviceName, updateFunction)),
		m_updateThread(Service::update)
	{
	}
	
	~Service()
	{
		m_updateThread.detach();
	}
	
	static void init(int argc, char **argv, const std::string& programName)
	{
		ros::init(argc, argv, programName);
	}

private :
	static void update()
	{
		ros::spin();
	}
	
	ros::NodeHandle m_node;
	ros::ServiceServer m_service;
	std::thread m_updateThread;
};

#endif
