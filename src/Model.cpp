#include "Model.hpp"

#include <boost/filesystem.hpp>
#include <urdf_parser/urdf_parser.h>

#include <envire_base_types/registration/TypeCreatorFactory.hpp>

#include <base-logging/Logging.hpp>

// TODO: add function or macro to generate the names for frames
// joints, links, visual, collisions, motor and etc

#define BASE_TYPES_NAMESPACE "envire::base_types::"
#define GEOMETRY_NAMESPACE "envire::base_types::geometry::"
#define JOINT_NAMESPACE "envire::base_types::joints::"
#define MOTOR_NAMESPACE "envire::base_types::motors::"
#define SENSOR_NAMESPACE "envire::base_types::sensors::"

namespace smurf_loader
{

    Model::Model() {}

    Model::~Model() {}

    const std::string& Model::getName() const
    {
        return name;
    }

    const std::string& Model::getPrefix() const
    {
        return prefix;
    }

    const std::string& Model::getSmurfFilePath() const
    {
        return smurfFilePathAbsolute;
    }

    const std::string& Model::getUrdfFilePath() const
    {
        return urdfFilePathAbsolute;
    }

    const std::string& Model::getRootFrame() const
    {
        return rootFrame;
    }

    void Model::loadFromSmurf(std::shared_ptr<envire::core::EnvireGraph> graph, const envire::core::FrameId &parentFrameId,
                              const std::string &filePath, const std::string &prefix)
    {
        this->prefix = prefix;
        // Load model from file
        boost::filesystem::path filePathAbsolute(boost::filesystem::absolute(filePath));
        std::string rootFolder = filePathAbsolute.parent_path().generic_string();
        std::string fileName = filePathAbsolute.filename().generic_string();

        smurfFilePathAbsolute = filePathAbsolute.string();

        urdfModel = parseFile(smurfMap, rootFolder, fileName, true);

        //std::cout << "---- config map: " << std::endl;
        //std::cout << smurfMap.toJsonString() << std::endl;

        // Get URDF file path
        configmaps::ConfigVector::iterator it;
        for (it = smurfMap["files"].begin(); it != smurfMap["files"].end(); ++it)
        {
            boost::filesystem::path path((std::string)(*it));
            if (path.extension().generic_string() == ".urdf")
            {
                urdfFilePathAbsolute = boost::filesystem::canonical(rootFolder / path).generic_string();
            }
        }

        // set model name
        if (smurfMap.hasKey("modelname"))
        {
            name = prefix + smurfMap["modelname"].toString();
        }

        if (graph->containsFrame(parentFrameId) == false)
        {
            LOG_ERROR_S << "The graph does not contain the frame with id: " << parentFrameId;
            return;
        }

        loadLinks(graph, parentFrameId);
        loadJoints(graph);
        loadMotors(graph);
        loadSensors(graph);
    }

    void Model::loadLinks(std::shared_ptr<envire::core::EnvireGraph> graph, const envire::core::FrameId &parentFrame) {

        // add root frame to the graph
        // TODO: add init pose
        envire::core::Transform initPose;
        initPose.setIdentity();

        // parse all links and add them into new frame in the graph
        for(std::pair<std::string, urdf::LinkSharedPtr> linkPair: urdfModel->links_)
        {
            // add new frame for the link
            envire::core::FrameId linkFrame = prefix + linkPair.first;
            graph->addFrame(linkFrame);

            // find root link, attach the root frame to parent frame with initPose transformation
            if (linkPair.first == urdfModel->getRoot()->name)
            {
                rootFrame = linkFrame;
                graph->addTransform(parentFrame, linkFrame, initPose);
            }

            // create link object to store it in the graph
            configmaps::ConfigMap linkMap;
            linkMap["name"] = linkPair.first;
            linkMap["type"] = "Link";

            std::string className(BASE_TYPES_NAMESPACE + linkMap["type"].toString());
            envire::core::ItemBase::Ptr item = envire::base_types::TypeCreatorFactory::createItem(className, linkMap);
            if (!item) {
                LOG_ERROR_S << "Can not add link " << linkMap["name"].toString()
                            << ", probably the link type " << linkMap["type"].toString() << " is not registered.";
                return;
            }
            graph->addItemToFrame(linkFrame, item);

            // load additional link information
            loadVisuals(graph, linkPair.second, linkFrame);
            loadCollisions(graph, linkPair.second, linkFrame);
            loadInertia(graph, linkPair.second, linkFrame);
        }
    }

    void Model::loadVisuals(std::shared_ptr<envire::core::EnvireGraph> graph, urdf::LinkSharedPtr link, envire::core::FrameId linkFrame) {
        // TODO: check if only one visual is available, will it be stored in array or not
        for(urdf::VisualSharedPtr visual : link->visual_array)
        {
            // add visual frame
            // TODO: if name of visual is empty
            envire::core::FrameId visualFrame = linkFrame + "_" + visual->name + "_" + "visual";
            graph->addFrame(visualFrame);

            envire::core::Transform tf = convertPoseToTransform(visual->origin);
            graph->addTransform(linkFrame, visualFrame, tf);

            // fill the config with visual information
            configmaps::ConfigMap visualMap;
            visualMap["name"] = visual->name;

            // set geometry information
            if (fillGeometryConfig(visual->geometry, visualMap) == false)
                return;

            // set material information
            urdf::MaterialSharedPtr urdfMaterial = visual->material;
            if (urdfMaterial != nullptr) {
                visualMap["material"]["name"] = visualFrame + "_" + urdfMaterial->name;
                visualMap["material"]["textureFilename"] = urdfMaterial->texture_filename;

                // diffuse color is set over urdf::Visual from urdf file
                visualMap["material"]["diffuseColor"]["r"] = urdfMaterial->color.r;
                visualMap["material"]["diffuseColor"]["g"] = urdfMaterial->color.g;
                visualMap["material"]["diffuseColor"]["b"] = urdfMaterial->color.b;
                visualMap["material"]["diffuseColor"]["a"] = urdfMaterial->color.a;

                // get additional material information from smurf config
                for (configmaps::ConfigVector::iterator it = smurfMap["materials"].begin(); it != smurfMap["materials"].end(); ++it)
                {
                    // TODO: add check if config keys exist
                    configmaps::ConfigMap &materialMap = *it;
                    if(materialMap["name"].toString() == urdfMaterial->name)
                    {
                        visualMap["material"]["ambientColor"] = materialMap["ambientColor"][0];
                        visualMap["material"]["specularColor"] = materialMap["specularColor"][0];
                        visualMap["material"]["shininess"] = materialMap["shininess"];
                    }
                }
            }

            // create and add into the graph envire item with the object corresponding to config type
            std::string className(GEOMETRY_NAMESPACE + visualMap["type"].toString());
            envire::core::ItemBase::Ptr item = envire::base_types::TypeCreatorFactory::createItem(className, visualMap);
            if (!item) {
                LOG_ERROR_S << "Can not add visual " << visualMap["name"].toString()
                            << ", probably the visual type " << visualMap["type"].toString() << " is not registered.";
                return;
            }
            item->setTag("visual");
            graph->addItemToFrame(visualFrame, item);
        }
    }

    void Model::loadCollisions(std::shared_ptr<envire::core::EnvireGraph> graph, urdf::LinkSharedPtr link, envire::core::FrameId linkFrame) {
        // TODO: check if only one collision is available, will it be stored in array or not
        for(urdf::CollisionSharedPtr collision : link->collision_array)
        {
            // add collision frame
            envire::core::FrameId collisionFrame = linkFrame + "_" + collision->name + "_" + "collision";;
            graph->addFrame(collisionFrame);

            envire::core::Transform tf = convertPoseToTransform(collision->origin);
            graph->addTransform(linkFrame, collisionFrame, tf);

            // fill the config with collision information
            configmaps::ConfigMap collisionMap;
            collisionMap["name"] = collision->name;

            // add geometry information
            if (fillGeometryConfig(collision->geometry, collisionMap) == false)
                return;

            // create and add into the graph envire item with the object corresponding to config type
            std::string className(GEOMETRY_NAMESPACE + collisionMap["type"].toString());
            envire::core::ItemBase::Ptr item = envire::base_types::TypeCreatorFactory::createItem(className, collisionMap);
            if (!item) {
                LOG_ERROR_S << "Can not add collision " << collisionMap["name"].toString()
                            << ", probably the collision type " << collisionMap["type"].toString() << " is not registered.";
                return;
            }
            item->setTag("collision");
            graph->addItemToFrame(collisionFrame, item);
        }
    }

    void Model::loadInertia(std::shared_ptr<envire::core::EnvireGraph> graph, urdf::LinkSharedPtr link, envire::core::FrameId linkFrame)
    {
        urdf::InertialSharedPtr urdfInetrial = link->inertial;
        if (urdfInetrial == nullptr)
        {
            return;
        }

        envire::core::FrameId inertiaFrame = linkFrame + "_inertia";
        graph->addFrame(inertiaFrame);

        envire::core::Transform tf = convertPoseToTransform(link->inertial->origin);
        graph->addTransform(linkFrame, inertiaFrame, tf);

        // fill the config with inertia information
        configmaps::ConfigMap inertiaMap;
        inertiaMap["name"] = link->name;
        inertiaMap["type"] = "Inertia";
        inertiaMap["mass"] = urdfInetrial->mass;
        inertiaMap["xx"] = urdfInetrial->ixx;
        inertiaMap["xy"] = urdfInetrial->ixy;
        inertiaMap["xz"] = urdfInetrial->ixz;
        inertiaMap["yy"] = urdfInetrial->iyy;
        inertiaMap["yz"] = urdfInetrial->iyz;
        inertiaMap["zz"] = urdfInetrial->izz;

        // create and add into the graph envire item with the object corresponding to config type
        std::string className(BASE_TYPES_NAMESPACE + inertiaMap["type"].toString());
        envire::core::ItemBase::Ptr item = envire::base_types::TypeCreatorFactory::createItem(className, inertiaMap);
        if (!item) {
            LOG_ERROR_S << "Can not add inertia " << inertiaMap["name"].toString()
                        << ", probably the inertia type " << inertiaMap["type"].toString() << " is not registered.";
            return;
        }
        graph->addItemToFrame(inertiaFrame, item);
    }

    void Model::loadMotors(std::shared_ptr<envire::core::EnvireGraph> graph) {
        for (configmaps::ConfigVector::iterator it = smurfMap["motors"].begin(); it != smurfMap["motors"].end(); ++it)
        {
            configmaps::ConfigMap &motorMap = *it;

            std::string jointName = motorMap["joint"].toString();
            envire::core::FrameId jointFrame(prefix + jointName + "_joint");

            if (!graph->containsFrame(jointFrame)) {
                LOG_ERROR_S << "Can not add motor " << motorMap["name"].toString()
                            << ", because there is no frame " << jointFrame << " in the graph.";
                return;
            }

            // create and add into the graph envire item with the object corresponding to config type
            std::string className(MOTOR_NAMESPACE + motorMap["type"].toString());
            envire::core::ItemBase::Ptr item = envire::base_types::TypeCreatorFactory::createItem(className, motorMap);
            if (!item) {
                LOG_ERROR_S << "Can not add motor " << motorMap["name"].toString()
                            << ", probably the motor type " << motorMap["type"].toString() << " is not registered.";
                return;
            }
            graph->addItemToFrame(jointFrame, item);
        }
    }

    void Model::loadSensors(std::shared_ptr<envire::core::EnvireGraph> graph) {
        for (configmaps::ConfigVector::iterator it = smurfMap["sensors"].begin(); it != smurfMap["sensors"].end(); ++it)
        {
            configmaps::ConfigMap &sensorMap = *it;

            std::string linkName = sensorMap["link"].toString();
            envire::core::FrameId linkFrame(prefix + linkName);

            if (!graph->containsFrame(linkFrame)) {
                LOG_ERROR_S << "Can not add sensor " << sensorMap["name"].toString()
                            << ", because there is no frame " << linkFrame << " in the graph.";
                return;
            }

            // create and add into the graph envire item with the object corresponding to config type
            std::string className(SENSOR_NAMESPACE + sensorMap["type"].toString());
            envire::core::ItemBase::Ptr item = envire::base_types::TypeCreatorFactory::createItem(className, sensorMap);
            if (!item) {
                LOG_ERROR_S << "Can not add sensor " << sensorMap["name"].toString()
                            << ", probably the sensor type " << sensorMap["type"].toString() << " is not registered.";
                return;
            }
            graph->addItemToFrame(linkFrame, item);
        }
    }

    bool Model::fillGeometryConfig(urdf::GeometrySharedPtr geometry, configmaps::ConfigMap &config)
    {
        switch (geometry->type)
        {
            case urdf::Geometry::BOX:
            {
                urdf::BoxSharedPtr urdfGeom = urdf::dynamic_pointer_cast<urdf::Box>(geometry);
                config["type"] = "Box";
                config["size"]["x"] = urdfGeom->dim.x;
                config["size"]["y"] = urdfGeom->dim.y;
                config["size"]["z"] = urdfGeom->dim.z;
                break;
            }
            case urdf::Geometry::CYLINDER:
            {
                urdf::CylinderSharedPtr urdfGeom = urdf::dynamic_pointer_cast<urdf::Cylinder>(geometry);
                config["type"] = "Cylinder";
                config["radius"] = urdfGeom->radius;
                config["length"] = urdfGeom->length;
                break;
            }
            case urdf::Geometry::MESH:
            {
                urdf::MeshSharedPtr urdfGeom = urdf::dynamic_pointer_cast<urdf::Mesh>(geometry);
                config["type"] = "Mesh";
                config["filename"] = urdfGeom->filename;
                config["scale"]["x"] = urdfGeom->scale.x;
                config["scale"]["y"] = urdfGeom->scale.y;
                config["scale"]["z"] = urdfGeom->scale.z;
                break;
            }
            case urdf::Geometry::SPHERE:
            {
                urdf::SphereSharedPtr urdfGeom = urdf::dynamic_pointer_cast<urdf::Sphere>(geometry);
                config["type"] = "Sphere";
                config["radius"] = urdfGeom->radius;
                break;
            }
            default:
            {
                LOG_ERROR_S << "Got unknown urdf geometry type";
                return false;
            }
        }

        return true;
    }


    void Model::loadJoints(std::shared_ptr<envire::core::EnvireGraph> graph)
    {
        // TODO: in urdf joints can have dynamics: friction and damping, add these properties
        for(std::pair<std::string, urdf::JointSharedPtr> jointPair: urdfModel->joints_)
        {
            urdf::JointSharedPtr joint = jointPair.second;
            envire::core::FrameId sourceFrame(prefix + joint->parent_link_name);
            envire::core::FrameId targetFrame(prefix + joint->child_link_name);
            envire::core::FrameId jointFrame;

            configmaps::ConfigMap jointMap;
            jointMap["name"] = joint->name;

            if (joint->type == urdf::Joint::FIXED)
            {
                // the fixed joint will be added to the target frame
                jointFrame = targetFrame;
                jointMap["type"] = std::string("Fixed");

            } else {
                // the dynamic joint will be added to the additional joint frame btw source und target frame
                jointFrame = envire::core::FrameId(prefix + joint->name + "_joint");
                graph->addFrame(jointFrame);

                // joint to child/target is described by the motion of joint
                envire::core::Transform initPose;
                initPose.setIdentity();
                graph->addTransform(jointFrame, targetFrame, initPose);

                if (joint->type == urdf::Joint::FLOATING)
                    jointMap["type"] = "Floating";
                else {
                    jointMap["axis"]["x"] = joint->axis.x;
                    jointMap["axis"]["y"] = joint->axis.x;
                    jointMap["axis"]["z"] = joint->axis.x;

                    switch(joint->type)
                    {
                        case urdf::Joint::REVOLUTE:
                        {
                            jointMap["type"] = "Revolute";
                            jointMap["minPosition"] = joint->limits->lower;
                            jointMap["maxPosition"] = joint->limits->upper;
                            jointMap["maxEffort"] = joint->limits->effort;
                            jointMap["maxVelocity"] = joint->limits->velocity;
                            break;
                        }
                        case urdf::Joint::CONTINUOUS:
                        {
                            jointMap["type"] = "Continuous";
                            jointMap["maxEffort"] = joint->limits->effort;
                            jointMap["maxVelocity"] = joint->limits->velocity;
                            break;
                        }
                        case urdf::Joint::PRISMATIC:
                        {
                            LOG_ERROR_S << "NOT IMPLEMENTED Model::loadJoints urdf::Joint::PRISMATIC";
                            return;
                        }
                        break;
                        default: {
                            LOG_ERROR_S << "Error, got unknown joint type";
                            return;
                        }
                    }
                }
            }

            // parent/source to joint is described by urdf origin
            envire::core::Transform parentToJoint = convertPoseToTransform(joint->parent_to_joint_origin_transform);
            graph->addTransform(sourceFrame, jointFrame, parentToJoint);

            // create and add into the graph envire item with the object corresponding to config type
            std::string className(JOINT_NAMESPACE + jointMap["type"].toString());
            envire::core::ItemBase::Ptr item = envire::base_types::TypeCreatorFactory::createItem(className, jointMap);
            if (!item) {
                LOG_ERROR_S << "Can not add joint " << jointMap["name"].toString()
                            << ", probably the joint type " << jointMap["type"].toString() << " is not registered.";
                return;
            }
            graph->addItemToFrame(jointFrame, item);

        }
    }

    envire::core::Transform Model::convertPoseToTransform(const urdf::Pose &pose)
    {
        base::Position position(pose.position.x,
                                pose.position.y,
                                pose.position.z);
        base::Orientation orientation(pose.rotation.w,
                                      pose.rotation.x,
                                      pose.rotation.y,
                                      pose.rotation.z);

        return envire::core::Transform(position, orientation);
    }

    urdf::ModelInterfaceSharedPtr Model::parseFile(configmaps::ConfigMap &map, const std::string &folderPath,
                                                   const std::string &fileName, bool expandURIs)
    {

        // TODO: add the check if rootFolder containes "/" at the end, trim all spaces behind
        std::string path = folderPath + "/"; // secure that path and file are combined correctly

        // parse yaml data and write to provided map, identify path to URDF file
        map.append(configmaps::ConfigMap::fromYamlFile(path + fileName, expandURIs));
        // map->toYamlFile("smurfparserdebug.yml");
        std::string urdfpath = "";
        configmaps::ConfigVector::iterator it;
        for (it = map["files"].begin(); it != map["files"].end(); ++it)
        {
            boost::filesystem::path filepath((std::string)(*it));
            if (filepath.extension().generic_string() == ".urdf")
            {
                urdfpath = path + filepath.generic_string();
            }
            else if (filepath.extension() == ".yml")
            {
                configmaps::ConfigMap tmpconfig =
                    configmaps::ConfigMap::fromYamlFile(path + filepath.generic_string());
                configmaps::ConfigMap::iterator mit = tmpconfig.begin();
                for (; mit != tmpconfig.end(); ++mit)
                {
                    if (mit->second.isVector())
                    {
                        configmaps::ConfigVector::iterator vit = mit->second.begin();
                        for (; vit != mit->second.end(); ++vit)
                        {
                            map[mit->first].append(*vit);
                        }
                    }
                    else
                    {
                        map[mit->first].appendMap(mit->second);
                    }
                }
            }
        }

        // parse URDF model and return
        fprintf(stderr, "  ...loading urdf data from %s.\n", urdfpath.c_str());
        urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdfpath);
        if (!model)
        {
            return urdf::ModelInterfaceSharedPtr();
        }
        return model;
    }

}