#include <gazebo/gazebo.hh>

namespace gazebo {
	class WorldPluginMyRobot : public WorldPlugin {
		public: WorldPluginMyRobot() : WorldPlugin() {
							printf("Hello world\n");
						}

						// should always be included since it receives information from the world file
		public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
						}
	};
	//register the plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)
}
