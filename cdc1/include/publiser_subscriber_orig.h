#ifndef PUBLISHER_SUBSCRIBER
#define PUBLISHER_SUBSCRIBER

#include <ros/ros.h>
#include <string>

template <typename PublishT, typename SubscribeT>
class PublisherSubscriber
{
   public:
      PublisherSubscriber() {}
      PublisherSubscriber(std::string publishTopicName, std::string subscribeTopicName, int queueSize)
      {
	      publisherObject  = nH.advertise<PublishT>(publishTopicName, queueSize) ;
	      subscriberObject = nH.advertise<SubscribeT>(subscribeTopicName, queueSize, &PublisherSubscriber::subscriberCallback, this) ;
      }
      void subscriberCallback(const typename SubscribeT::ConstPtr& receivedMsg) ;
//    void subscriberCallback(const typename SubscribeT& receivedMsg) ;

   protected:
      ros::Subscriber subscriberObject ;
      ros::Publisher  publisherObject ;
      ros::NodeHandle nH ;
} ;

#endif

