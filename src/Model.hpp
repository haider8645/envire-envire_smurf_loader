#ifndef MODEL_HPP
#define MODEL_HPP

#include <configmaps/ConfigData.h>
#include <urdf_world/types.h>
#include <urdf_model/types.h>
#include <urdf_model/pose.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Transform.hpp>

// TODO: can we use smurf loader for urdf too: loadFromUrdf
// TODO: add option to load full graph urdf structure, so each link willbe place in extra frame
// or compacte, so fixed link will be placed in the same frame as parent link
// TODO: do we need link object? is it not enough to have frame to represent a link?
// TODO: does smurf can diffine additional link, visual, collision over smurf files?
// TODO: where the fixed joint should be? in parent frame or in child frame?
// the urdf defines joint transformation as btw parent and joint origin
// TODO: add material into base envire types property to Geometry
// TODO: add urdf loader, espeically it is relevant for motors, since all joints information should goes to the motor than

namespace envire {
    namespace smurf_loader
    {
        extern const std::string base_types_namespace;
        extern const std::string geometry_namespace;
        extern const std::string joint_namespace;
        extern const std::string motor_namespace;
        extern const std::string sensor_namespace;

        class Model
        {
        public:

            Model();

            ~Model();

            const std::string& getName() const;

            const std::string& getPrefix() const;

            const std::string& getRootFrame() const;

            const std::string& getSmurfFilePath() const;

            const std::string& getUrdfFilePath() const;

            void loadFromSmurf(std::shared_ptr<envire::core::EnvireGraph> graph,
                                const envire::core::FrameId &parentFrame,
                                const std::string &filePath,
                                const base::Position &position = base::Position(0, 0, 0),
                                const base::Orientation &orientation = base::Orientation(1, 0, 0, 0),
                                const std::string &prefix = "");

        private:
            std::string name;
            std::string prefix;
            std::string smurfFilePathAbsolute;
            std::string urdfFilePathAbsolute;
            urdf::ModelInterfaceSharedPtr urdfModel;

            configmaps::ConfigMap smurfMap;

            // TODO: check if we need this
            std::string rootFrame;


            static urdf::ModelInterfaceSharedPtr parseFile(configmaps::ConfigMap &map, const std::string &folderPath,
                                                            const std::string &fileName, bool expandURIs);


            void loadFromSmurf(const std::string &filePath, const std::string &prefix = "");

            void loadStructure(std::shared_ptr<envire::core::EnvireGraph> graph, const envire::core::FrameId &parentFrame);
            void loadLinks(std::shared_ptr<envire::core::EnvireGraph> graph);
            void loadVisuals(std::shared_ptr<envire::core::EnvireGraph> graph, urdf::LinkSharedPtr link, envire::core::FrameId linkFrame);
            void loadCollisions(std::shared_ptr<envire::core::EnvireGraph> graph, urdf::LinkSharedPtr link, envire::core::FrameId linkFrame);
            void loadInertia(std::shared_ptr<envire::core::EnvireGraph> graph, urdf::LinkSharedPtr link, envire::core::FrameId linkFrame);
            void loadJoints(std::shared_ptr<envire::core::EnvireGraph> graph);
            void loadMotors(std::shared_ptr<envire::core::EnvireGraph> graph);
            void loadSensors(std::shared_ptr<envire::core::EnvireGraph> graph);

            bool fillGeometryConfig(urdf::GeometrySharedPtr geometry, configmaps::ConfigMap &config);

            static envire::core::Transform convertPoseToTransform(const urdf::Pose &pose);
        };
    }
}
#endif // MODEL_HPP
