#include <gazebo/gazebo.hh>

namespace gazebo {
  class WorldPluginUdacRobot : public WorldPlugin {
    public: WorldPluginUdacRobot() : WorldPlugin() {
              printf("Welcome to UdacityWord world!\n");
            }
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){}
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginUdacRobot)
}
