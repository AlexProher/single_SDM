// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.
// =============================================================================


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
#include "chrono/core/ChRandom.h"

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

void CreateBrick(ChSystemNSC& sys, ChVector3d pos, double dimX, double dimY, double dimZ) {
    auto brick_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto brick_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    auto brick = chrono_types::make_shared<ChBodyEasyBox>(dimX, dimY, dimZ, 1, true, true, brick_mat);
    brick->SetPos(pos);
    brick->SetFixed(true);
    brick->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/redwhite.png"));
    sys.AddBody(brick);
}

void CreateBumper(ChSystemNSC& sys, ChVector3d pos, double r, double h) {
    auto bumper_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto bumper_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    auto bumper = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis(2), r, h, 1, true, true, bumper_mat);
    bumper->SetPos(pos);
    bumper->SetFixed(true);
    bumper->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/redwhite.png"));
    sys.AddBody(bumper);
}

void AddFallingItems(ChSystemNSC& sys, double posX) {
    // Shared contact materials for falling objects
    auto box_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto cyl_mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // Create falling rigid bodies (spheres and boxes etc.)
    for (int bi = 0; bi < 59; bi++) {
        double r = 0.2 * ChRandom::Get();
        auto cylBody = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis(2),  //
            r,
            2,  // radius, height
            100,        // density
            cyl_mat     // contact material
        );
        cylBody->SetPos(ChVector3d(posX + double(bi)/5 + r, 0.2, 0));
        cylBody->SetFixed(true);
        cylBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/redwhite.png"));
        sys.Add(cylBody);
    }
}

int main(int argc, char* argv[]) {

    bool control = false;

    Document config;
    ReadFileJSON("../../sourceFiles/configuration.json", config);
    //ReadFileJSON("../sourceFiles/configuration.json", config);

    assert(config.HasMember("Position"));
    //config.ParseStream(isw);
    char val;
    try {

        ChSystemNSC sys;


        ChCollisionSystem::Type collision_type = ChCollisionSystem::Type::BULLET;
        sys.SetCollisionSystemType(collision_type);

        // Create a Chrono physical system

        MySystem newSystem(config);
        newSystem.AddSystem(sys);

        CreateBrick(sys, 
                    ChVector3d(config["Brick1"]["Position"]["x"].GetDouble(),
                                config["Brick1"]["Position"]["y"].GetDouble(),
                                config["Brick1"]["Position"]["z"].GetDouble()),
                    config["Brick1"]["Dimention"]["x"].GetDouble(),
                    config["Brick1"]["Dimention"]["y"].GetDouble(),
                    config["Brick1"]["Dimention"]["z"].GetDouble());
        CreateBrick(sys,
            ChVector3d(config["Brick2"]["Position"]["x"].GetDouble(),
                        config["Brick2"]["Position"]["y"].GetDouble(),
                        config["Brick2"]["Position"]["z"].GetDouble()),
                    config["Brick2"]["Dimention"]["x"].GetDouble(),
                    config["Brick2"]["Dimention"]["y"].GetDouble(),
                    config["Brick2"]["Dimention"]["z"].GetDouble());
        CreateBumper(sys, 
                    ChVector3d(config["Bumper"]["Position"]["x"].GetDouble(),
                                config["Bumper"]["Position"]["y"].GetDouble(),
                                config["Bumper"]["Position"]["z"].GetDouble()),
                    config["Bumper"]["Dimention"]["r"].GetDouble(),
                    config["Bumper"]["Dimention"]["h"].GetDouble());

        AddFallingItems(sys, config["Noize"]["x"].GetDouble());
        
        control = config["Control"].GetBool();

        // Add a socket framework object
        ChSocketFramework socket_tools;

        //// Create the cosimulation interface:

        int nInp = 1;
        int nOut = 3;

        ChSocketCommunication cosimul_interface(socket_tools,
                                nInp,   // n.input values from Simulink
                                nOut);  // n.output values to Simulink

        if (control) {

            //// 4) Wait client (Simulink) to connect...
            std::cout << " *** Waiting Simulink to start... ***\n"
                << "(load 'data/cosimulation/test_cosim_hydraulics.mdl' in Simulink and press Start...)\n"
                << std::endl;

            int PORT_NUMBER = 50009;

            cosimul_interface.WaitConnection(PORT_NUMBER);
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

        //// Here the 'dt' must be the same of the sampling period that is
        //// entered in the CEcosimulation block

        double dt = 0.001;
        double actCamPosX = config["Camera"]["x"].GetDouble();
        double actCamPosY = config["Camera"]["y"].GetDouble();
        double actCamPosZ = config["Camera"]["z"].GetDouble();

        // Optionally, set color and/or texture for visual assets

        // 4 - Create the Irrlicht visualization system
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

        myfile << "time\tx_pos\tx_vel\tx_angle\tdx_angle\n";

        while (vis->Run()) 
        //while (time < 10)
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


            //data_out(3) = -cart.getBodyVel().x();
            //data_out(2) = -cart.getBodyPos().x();
            //data_out(1) = cart.getSphereAngleDt().z();
            //data_out(0) = newSystem.GetWheelPos().y() - config["Wheel"]["rWheel"].GetDouble();
            data_out(0) = newSystem.GetWheelPos().y();
            data_out(1) = newSystem.GetBodyPos().y()-config["SD"]["base"].GetDouble();
            data_out(2) = newSystem.GetWheelPos().x();
            std::cout << "--- Y wheelPos: " << data_out(0)
                    << "--- Y bodyPosRel: " << data_out(1)
                     << "--- data_in: " << data_in(0)
                    //<< "--- camPosY: " << vis->GetActiveCamera()->getAbsolutePosition().Y
                    //<< "--- camPosZ: " << vis->GetActiveCamera()->getAbsolutePosition().Z
                    << std::endl;



            // Spin in place to maintain soft real-time
            //realtime_timer.Spin(dt);
            //myfile << time;
            //myfile << '\t';
            //myfile << data_out(2);
            //myfile << '\t';
            //myfile << data_out(3);
            //myfile << '\n';
            //myfile << data_out(0);
            //myfile << '\n';
            //myfile << data_out(1);
            //myfile << '\n';
            //myfile << data_in(0);

            if (control) {
                // std::cout << "Send" << std::endl;
                cosimul_interface.SendData(time, data_out);  // --> to Simulink
                // std::cout << "Receive" << std::endl;
                cosimul_interface.ReceiveData(histime, data_in);  // <-- from Simulink
                //newSystem.SetWheelPos(ChVector3d(newSystem.GetWheelPos().x(),
                //                                newSystem.GetWheelPos().y() + data_in(0),
                //                                newSystem.GetWheelPos().z()));
                newSystem.UpdateActForce(data_in(0));
                //std::cout << "--- time: " << time << std::endl;
            }

        }
        myfile.close();

    }
    catch (std::exception exception) {
        std::cerr << " ERRROR with socket system:\n" << exception.what() << std::endl;

    }

    std::cout << "Press any key";
    std::cin >> val;
    return 0;
}
