#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/utils/ChSocketCommunication.h"
#include "MySystem.h"

#include <fstream>

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"


// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::utils;
using namespace rapidjson;

void ReadFileJSON(const std::string& filename, Document& d) {
    std::ifstream ifs(filename);
    if (!ifs.good()) {
        std::cerr << "ERROR: Could not open JSON file: " << filename << std::endl;
    }
    else {
        IStreamWrapper isw(ifs);
        d.ParseStream<ParseFlag::kParseCommentsFlag>(isw);
        if (d.IsNull()) {
            std::cerr << "ERROR: Invalid JSON file: " << filename << std::endl;
        }
    }
}

int main(int argc, char* argv[]) {

    bool control = false;

    Document config;
    ReadFileJSON("../../sourceFiles/configuration.json", config);
    //ReadFileJSON("../sourceFiles/configuration.json", config);    // in case of Debug

    try {

        ChSystemNSC sys;

        ChCollisionSystem::Type collision_type = ChCollisionSystem::Type::BULLET;
        sys.SetCollisionSystemType(collision_type);

        // Create a Chrono physical system

        MySystem newSystem;

        newSystem.BuildConfig(config);
        newSystem.AddSystem(sys);


        // Add Obstacles in the system

        newSystem.CreateBrick(sys,
                    ChVector3d(config["Brick1"]["Position"]["x"].GetDouble(),
                                config["Brick1"]["Position"]["y"].GetDouble(),
                                config["Brick1"]["Position"]["z"].GetDouble()),
                            config["Brick1"]["Dimention"]["x"].GetDouble(),
                            config["Brick1"]["Dimention"]["y"].GetDouble(),
                            config["Brick1"]["Dimention"]["z"].GetDouble());
                    
        newSystem.CreateBrick(sys,
                    ChVector3d(config["Brick2"]["Position"]["x"].GetDouble(),
                                config["Brick2"]["Position"]["y"].GetDouble(),
                                config["Brick2"]["Position"]["z"].GetDouble()),
                            config["Brick2"]["Dimention"]["x"].GetDouble(),
                            config["Brick2"]["Dimention"]["y"].GetDouble(),
                            config["Brick2"]["Dimention"]["z"].GetDouble());
        newSystem.CreateBumper(sys,
                    ChVector3d(config["Bumper"]["Position"]["x"].GetDouble(),
                                config["Bumper"]["Position"]["y"].GetDouble(),
                                config["Bumper"]["Position"]["z"].GetDouble()),
                            config["Bumper"]["Dimention"]["r"].GetDouble(),
                            config["Bumper"]["Dimention"]["h"].GetDouble());

        newSystem.AddRandomCylinders(sys, config["Noize"]["x"].GetDouble(),
                                config["Noize"]["dimMax"].GetDouble(), 
                                config["Noize"]["N"].GetDouble(),
                                config["Noize"]["distFactor"].GetDouble());

        // Get parameters for Simulation
        // Here the 'dt' must be the same of the sampling period that is
        // entered in the CEcosimulation block

        control = config["General"]["Control"].GetBool();
        double dt = config["General"]["Ts"].GetDouble();

        // Get values for camera position wrt to body
        double actCamPosX = config["Camera"]["x"].GetDouble();
        double actCamPosY = config["Camera"]["y"].GetDouble();
        double actCamPosZ = config["Camera"]["z"].GetDouble();

        // Add a socket framework object

        ChSocketFramework socket_tools;

        //// Create the cosimulation interface:

        int nInp = 1;
        int nOut = 3;

        ChSocketCommunication cosimul_interface(socket_tools,
                                nInp,   // n.input values from Simulink
                                nOut);  // n.output values to Simulink

        if (control) {

            //// Wait client (Simulink) to connect...
            std::cout << " *** Waiting Simulink to start... ***\n"
                << "(load 'data/cosimulation/test_cosim_hydraulics.mdl' in Simulink and press Start...)\n"
                << std::endl;

            int PORT_NUMBER = 50009;

            cosimul_interface.WaitConnection(PORT_NUMBER);
        }
        else {
            std::cout << "To start press Enter";
            getchar();
        }
        
        // Prepare the two column vectors of data that will be swapped
        // back and forth between Chrono and Simulink. In detail we will
        // - receive 1 variable from Simulink (the hydraulic cylinder force)
        // - send 2 variables to Simulink (the hydraulic cylinder velocity and displacement)
        ChVectorDynamic<double> data_in(nInp);
        ChVectorDynamic<double> data_out(nOut);
        data_in.setZero();
        data_out.setZero();

        double mytime = 0;
        double histime = 0;

        // Optionally, set color and/or texture for visual assets

        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("A simple pendulum example");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(actCamPosX, actCamPosY, actCamPosZ), newSystem.GetBodyPos());
        vis->AddTypicalLights();

        ChRealtimeStepTimer realtime_timer;

        double time = 0;

        std::ofstream myfile("example.txt");
        if (!myfile.is_open()) {
            std::cout << "Unable to open file";
            return 0;
        }

        myfile << "time\tWheelYpos\tWheelXpos\tBodyYpos\tControlOut\n";

        while (vis->Run()) 
        {

            // Render scene
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            actCamPosX = newSystem.GetBodyPos().x();

            vis->UpdateCamera(ChVector3d(actCamPosX, actCamPosY, actCamPosZ), newSystem.GetBodyPos());
            tools::drawSpring(vis.get(), 1, newSystem.GetBodyPos(), newSystem.GetWheelPos(),
                                        ChColor(1, 1, 1), 80, 10, true);

            // Perform the integration stpe
            sys.DoStepDynamics(dt);
            time += dt;

            data_out(0) = newSystem.GetWheelPos().y();
            data_out(1) = newSystem.GetBodyPos().y()-config["SD"]["base"].GetDouble();
            data_out(2) = newSystem.GetWheelPos().x();



            std::cout << "--- Y wheelPos: " << data_out(0)
                        << "--- X wheelPos: " << data_out(2)
                        << "--- Y bodyPosRel: " << data_out(1)
                        << "--- Control out: " << data_in(0)
                        << std::endl;

            myfile << time;
            myfile << '\t';
            myfile << data_out(0);
            myfile << '\t';
            myfile << data_out(2);
            myfile << '\t';
            myfile << data_out(1);
            myfile << '\t';
            myfile << data_in(0);
            myfile << '\n';

            if (control) {
                cosimul_interface.SendData(time, data_out);         // --> to Simulink
                cosimul_interface.ReceiveData(histime, data_in);    // <-- from Simulink
                newSystem.UpdateActForce(data_in(0));               // Apply Force to Suspension
            }

        }
        myfile.close();

    }
    catch (std::exception exception) {
        std::cerr << " ERRROR with socket system:\n" << exception.what() << std::endl;

    }

    std::cout << "To finish press Enter";;
    getchar();
    return 0;
}
