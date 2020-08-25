#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/Collision.hh>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <gazebo/common/common.hh>

#include <sstream>

#include <vector> 

#include <algorithm>    // std::lower_bound

namespace gazebo
{
  class WorldUuvPlugin : public WorldPlugin
  {

    /// \brief Pointer to the Gazebo world.
    private: physics::WorldPtr world;

    /// \brief Pointer to the plug model.
    private: physics::ModelPtr plugModel;

    /// \brief Pointer to the plug link.
    private: physics::LinkPtr plugLink;

    /// \brief Pointer to the socket model.
    private: physics::ModelPtr socketModel;

    /// \brief Pointer to the hollow tube like structure that receives the plug.
    private: physics::LinkPtr tubeLink;

    /// \brief Pointer to the sensor plate in the socket model that senses when
    /// the plug is inserter.
    private: physics::LinkPtr sensorPlate;

    /// \brief Pinter to the prismatic joint formed between the plug and the 
    /// socket.
    private: physics::JointPtr prismaticJoint;

    /// \brief Boolean that indicates weather the prismatic joint is created.
    private: bool joined = false;
    
    /// \brief Boolean that indicates weather the plug and socket are fixed.
    private: bool locked = false;

    /// \brief Pointer to the update event connection.
    private: gazebo::event::ConnectionPtr updateConnection;
    
    /// \brief Vector that contains forces (For the purpose of averaging).
    private: std::vector<double> forcesBuffer;
    
    /// \brief Time stamps associated with forces in the above vector.
    private: std::vector<common::Time> timeStamps;

    /// \brief Time the plug and socket has been in alignment.
    private: common::Time alignmentTime = 0;
    
    /// \brief Time the plug and the socket has been freed.
    /// Used to put some buffer between unfreezing and another possible mating.
    private: common::Time unfreezeTimeBuffer = 0;
    
    /// \brief Concatenates/trims forcesBuffer and timeStamps vectors to 
    /// include only the last trimDuration.
     /// \param[in] trimDuration Duration over which to trim the vectors.
    void trimForceVector(double trimDuration);

    /// \brief Calculates the average of the forcesBuffer vector.
    /// \return Average of the forcesBuffer vector.
    double movingTimedAverage();

    /// \brief Appends another force reading to the forcesBuffer vector.
    void addForce(double force);

    /// \brief Constructor.
    public: WorldUuvPlugin();

    /// Documentation inherited.
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Locks the prismatic joint.
    public: void lockJoint(physics::JointPtr prismaticJoint);

    /// \brief Release the plug from the joint.
    public: void unfreezeJoint(physics::JointPtr prismaticJoint);

    /// \brief Check that plug and socket are aligned in the roll orientation.
    /// \return Return true of aligned.
    private: bool checkRollAlignment(bool verbose = false);

    /// \brief Check that plug and socket are aligned in the pitch orientation.
    /// \return Return true if aligned.
    private: bool checkPitchAlignment(bool verbose = false);

    /// \brief Check that plug and socket are aligned in the yaw orientation.
    /// \return Return true if aligned.
    private:bool checkYawAlignment(bool verbose = false);

    /// \brief Check that plug and socket are aligned in the all orientations.
    /// \return Return true if aligned.
    private: bool checkRotationalAlignment();

    /// \brief Check if plug and socket have the same altitude.
    /// \return Return true if on same altitude.
    private: bool checkVerticalAlignment(bool verbose = false);

    /// \brief Check if plug and socket have the same orientation and altitude.
    /// \return Return true if same r,p,y and z.
    private: bool isAlligned(bool verbose = false);

    /// \brief Measure Euclidean distance between plug an socket.
    /// \return Return true if plug is close to the socket.
    private: bool checkProximity(bool verbose = false);

    /// \brief Creates the prismatic joint between the socket and plug.
    private: void construct_joint();
        
    /// \brief Distroys the prismatic joint between the socket and the plug.
    private: void remove_joint();

    /// \brief Calculates the average force exerted by contact2 on contact 1.
    /// \return Average force exerted by contact2 on contact1.
    public: bool averageForceOnLink(std::string contact1, std::string contact2);

    /// \brief Determine of Electrical Plug is pushing against electrical socket.
    /// \return boolean weather the plug is pushing against the socket.
    public: bool isPlugPushingSensorPlate(float averageForceThreshold = 50, int numberOfDatapointsThresh = 10);

    /// \brief Determine of Electrical Plug is pushing against electrical socket.
    /// \return boolean weather the plug is pushing against the socket.
    public: bool isEndEffectorPushingPlug(float averageForceThreshold = 200, int numberOfDatapointsThresh = 10);

    /// \brief Gets the collision index between two links.
    /// \return Collision index between contact1 and contact2.
    public: int getCollisionBetween(std::string contact1, std::string contact2);

    /// \brief Callback executed at every physics update.
    public: void Update();
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
} // namespace gazebo
