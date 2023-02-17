#include <iostream>
#include <boost/program_options.hpp>
#include <fstream>

#include "Model.hpp"
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphDrawing.hpp>


void parse_args(int argc, char** argv, std::string& smurf_file, std::string& what, std::string& out)
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help",      "Produce help message")
            ("out",       po::value<std::string>(&out), "Output YAML file with the resulting information")
            ;
    //These arguments are positional and thuis should not be show to the user
    po::options_description hidden("Hidden");
    hidden.add_options()
            ("smurf_file", po::value<std::string>(&smurf_file)->required(), "Input SMURF file")
            ("what",       po::value<std::string>(&what), "Information to dump. Valid values are: 'sensors', 'motors'. If not specified, all available information is dumped.")
            ;

    //Collection of all arguments
    po::options_description all("Allowed options");
    all.add(desc).add(hidden);

    //Collection of only the visible arguments
    po::options_description visible("Allowed options");
    visible.add(desc);

    //Define th positional arguments (they refer to the hidden arguments)
    po::positional_options_description p;
    p.add("smurf_file", 1);
    p.add("what", 1);

    //Parse all arguments, but ....
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(all).positional(p).run(), vm);
    //po::notify(vm);

    if (vm.count("help") || argc < 2)
    {
        //... but display only the visible arguments
        std::cout << "smurf_dump allows to dump a parts of a SMURF model, which is usually distributed over multiple files, to a single file or the terminal.\n" << std::endl;
        std::cout << "USAGE: \n\t" << argv[0] << " SMURF_FILE [sensors|motors]\n"<<std::endl;
        desc.print(std::cout);
        exit(EXIT_FAILURE);
    }
    po::notify(vm);
}

int main(int argc, char** argv)
{
    std::string smurf_file = "", what = "", out = "";
    parse_args(argc, argv, smurf_file, what, out);

    std::shared_ptr<envire::core::EnvireGraph> graph = std::make_shared<envire::core::EnvireGraph>();
    envire::core::FrameId rootFrame("INIT_FRAME");
    graph->addFrame(rootFrame);

    envire::smurf_loader::Model model;
    model.loadFromSmurf(graph, rootFrame, smurf_file);

    configmaps::ConfigMap mmap;
    configmaps::ConfigVector vect;

    std::cout << "modelname: " << model.getName() << std::endl;

    if(what == ""){
        std::string fp = model.getUrdfFilePath();
        mmap["urdf"] = fp;
    }
    else if(what == "urdf"){
        std::cout << model.getUrdfFilePath();
        return 0;
    }

    /*if(what == "" || what == "sensors"){
        std::vector<smurf::Sensor *> sensors = model.getSensors();
        for(smurf::Sensor* item : sensors)
        {
            configmaps::ConfigMap map = item -> getMap();
            vect.append(map);
        }
        mmap["sensors"] = vect;
        vect.clear();
    }

    if(what == "" || what == "motors"){
        std::vector<smurf::Motor *> motors = model.getMotors();
        for(smurf::Motor* item : motors)
        {
            configmaps::ConfigMap map = item->getMotorMap();
            vect.append(map);
        }
        mmap["motors"] = vect;
        vect.clear();
    }*/

     std::string out_dot_file = "test.dot";
    envire::core::GraphDrawing::write(*graph, out_dot_file);

    std::string mapString = mmap.toYamlString();
    if(out == ""){
        std::cout << mapString << std::endl;
    }else{
        std::ofstream fout;
        fout.open (out);
        fout << mapString;
        fout.close();
    }
    return 0;
}
