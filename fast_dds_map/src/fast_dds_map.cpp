#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
// DDS header
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>

#include "OccupancyGrid_mapPubSubTypes.h"

using namespace eprosima::fastdds::dds;

class OccupancyGridPublisher
{
public:
    OccupancyGrid_map Grid_map;

private:
    DomainParticipant *participant_;

    Publisher *publisher_;

    Topic *topic_;

    DataWriter *writer_;

    TypeSupport type_;

    class PubListener : public DataWriterListener
    {
    public:
        PubListener()
            : matched_(0)
        {
        }

        ~PubListener() override
        {
        }

        void on_publication_matched(
            DataWriter *,
            const PublicationMatchedStatus &info) override
        {
            if (info.current_count_change == 1)
            {
                matched_ = info.total_count;
                std::cout << "Publisher matched." << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                matched_ = info.total_count;
                std::cout << "Publisher unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                          << " is not a valid value for PublicationMatchedStatus current count change." << std::endl;
            }
        }

        std::atomic_int matched_;

    } listener_;

public:
    OccupancyGridPublisher()
        : participant_(nullptr), publisher_(nullptr), topic_(nullptr), writer_(nullptr), type_(new OccupancyGrid_mapPubSubType())
    {
    }

    virtual ~OccupancyGridPublisher()
    {
        if (writer_ != nullptr)
        {
            publisher_->delete_datawriter(writer_);
        }
        if (publisher_ != nullptr)
        {
            participant_->delete_publisher(publisher_);
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    //! Initialize the publisher
    bool init()
    {
        Grid_map.resolution(0);
        Grid_map.width(0);
        Grid_map.height(0);
        Grid_map.position_x(0);
        Grid_map.position_y(0);
        Grid_map.position_z(0);
        Grid_map.orientation_x(0);
        Grid_map.orientation_y(0);
        Grid_map.orientation_z(0);
        Grid_map.orientation_w(0);
        Grid_map.data().clear();

        DomainParticipantQos participantQos;
        participantQos.name("Participant_publisher");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // Register the Type
        type_.register_type(participant_);

        // Create the publications Topic
        topic_ = participant_->create_topic("OccupancyGridMapTopic", "OccupancyGrid_map", TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // Create the Publisher
        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

        if (publisher_ == nullptr)
        {
            return false;
        }

        // Create the DataWriter
        writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &listener_);

        if (writer_ == nullptr)
        {
            return false;
        }
        return true;
    }

    //! Send a publication
    bool publish()
    {
        if (listener_.matched_ > 0)
        {
            writer_->write(&Grid_map);
            return true;
        }
        return false;
    }
};

OccupancyGridPublisher *mypub = nullptr;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    ROS_INFO("Received map with width: %d and height: %d ", msg->info.width, msg->info.height);
    mypub->Grid_map.resolution(msg->info.resolution);
    mypub->Grid_map.width(msg->info.width);
    mypub->Grid_map.height(msg->info.height);
    mypub->Grid_map.position_x(msg->info.origin.position.x);
    mypub->Grid_map.position_y(msg->info.origin.position.y);
    mypub->Grid_map.position_z(msg->info.origin.position.z);
    mypub->Grid_map.orientation_x(msg->info.origin.orientation.x);
    mypub->Grid_map.orientation_y(msg->info.origin.orientation.y);
    mypub->Grid_map.orientation_z(msg->info.origin.orientation.z);
    mypub->Grid_map.orientation_w(msg->info.origin.orientation.w);
    mypub->Grid_map.data().clear();
    for (int i = 0; i < msg->info.height; ++i)
        for (int j = 0; j < msg->info.width; ++j)
            mypub->Grid_map.data()
                .push_back(msg->data[i * msg->info.height + j]);
    ROS_INFO("data size : %ld ", mypub->Grid_map.data().size());
    mypub->publish();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fast_dds_map");
    ros::NodeHandle n;

    mypub = new OccupancyGridPublisher();
    if (!mypub->init())
    {
        std::cout << "OccupancyGridPublisher init failed!" << std::endl;
        return -1;
    }

    ros::Subscriber sub = n.subscribe("/map", 1000, mapCallback);

    ros::spin();

    return 0;
}
