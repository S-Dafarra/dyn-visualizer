/*
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Visualizer.h>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <URDFdir.h>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <fstream>

int main()
{
    bool isWindows = false;
#ifdef _WIN32
    isWindows = true;
#endif

    const char * env_var_value = std::getenv("GAZEBO_MODEL_PATH");

    std::stringstream env_var_string(env_var_value);

    std::string individualPath;
    std::vector<std::string> pathList;

    while(std::getline(env_var_string, individualPath, isWindows ? ';' : ':'))
    {
       pathList.push_back(individualPath);
    }

    auto cleanPathSeparator = [isWindows](const std::string& filename)->std::string
    {
        std::string output = filename;
        char pathSeparator = isWindows ? '\\' : '/';
        char wrongPathSeparator = isWindows ? '/' : '\\';

        for (size_t i = 0; i < output.size(); ++i)
        {
            if (output[i] == wrongPathSeparator)
            {
                output[i] = pathSeparator;
            }
        }

        return filename;

    };

    auto isFileExisting = [cleanPathSeparator](const std::string& filename)->bool
    {
        if (FILE *file = fopen(cleanPathSeparator(filename).c_str(), "r")) {
            fclose(file);
            return true;
        } else {
            return false;
        }
    };

    auto getFilePath = [isFileExisting](const std::string& filename, const std::string& prefixToRemove, const std::vector<std::string>& paths)
    {
        if(isFileExisting(filename))
        {
            return filename;
        }

        if (filename.substr(0, prefixToRemove.size()) == prefixToRemove)
        {
            std::string filename_noprefix = filename;
            filename_noprefix.erase(0, prefixToRemove.size());
            for (size_t i = 0; i < paths.size(); ++i)
            {
                std::string testPath;
                testPath = paths[i] + filename_noprefix;
                if (isFileExisting(testPath))
                {
                    return testPath;
                }
            }

        }

        return filename; //By default return the input;
    };


    iDynTree::ModelLoader modelLoader;
    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName("model.urdf");
    modelLoader.loadModelFromFile(pathToModel);

    iDynTree::Model loadedModel = modelLoader.model();
    std::vector<std::vector<iDynTree::SolidShape*>>& shapesContainer = loadedModel.visualSolidShapes().getLinkSolidShapes();

    for (size_t i = 0; i < shapesContainer.size(); ++i)
    {
        std::vector<iDynTree::SolidShape*>& shapes = shapesContainer[i];

        for (size_t s = 0; s < shapes.size(); ++s)
        {
            iDynTree::SolidShape* shape = shapes[s];
            if (shape->isExternalMesh())
            {
                iDynTree::ExternalMesh* mesh = shape->asExternalMesh();
                std::string originalPath = mesh->getFilename();
                std::string otherMeshFile = getFilePath(originalPath, "package:/", pathList);
                mesh->setFilename(otherMeshFile);
                iDynTree::Material material;
                iDynTree::Vector4 color;
                color[0] = 0.0; //r
                color[1] = 0.0; //g
                color[2] = 0.0; //b
                color[3] = 1.0; //a
                material.setColor(color);
                mesh->setMaterial(material);
            }
        }
    }


    iDynTree::Visualizer viz;
    iDynTree::VisualizerOptions options, textureOptions;
//    options.winWidth = 1920;
//    options.winHeight = 1080;
    textureOptions.winWidth = 400;
    textureOptions.winHeight = 400;

    viz.init(options, textureOptions);
//    viz.camera().setPosition(iDynTree::Position(2.0, 0.5, 0.5));
//    viz.camera().setTarget(iDynTree::Position(0.4, 0.0, 0.5));
    viz.camera().setPosition(iDynTree::Position(2.0, 0.0, 1.0));
    viz.camera().setTarget(iDynTree::Position(0.0, 0.0, 0.0));
    viz.camera().animator()->enableMouseControl(true);

    double sqrt2 = std::sqrt(2.0);
    viz.enviroment().lightViz("sun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));
//    viz.enviroment().setFloorGridColor(iDynTree::ColorViz(0.0, 1.0, 0.0, 1.0));


    viz.textureEnviroment().lightViz("sun").setDirection(iDynTree::Direction(-0.5/sqrt2, 0, -0.5/sqrt2));
    viz.textureEnviroment().setElementVisibility("floor_grid", true);
    viz.textureEnviroment().setElementVisibility("world_frame", false);
    viz.textureEnviroment().setBackgroundColor(iDynTree::ColorViz(0.0, 0.0, 0.0, 0.0));
    viz.textureEnviroment().setFloorGridColor(iDynTree::ColorViz(0.0, 1.0, 0.0, 1.0));


    viz.addModel(loadedModel, "robot");

    yarp::sig::FlexImage image;
    image.setPixelCode(VOCAB_PIXEL_RGBA);
    image.resize(textureOptions.winWidth, textureOptions.winHeight);

    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }

    yarp::os::BufferedPort<yarp::sig::FlexImage> imagePort;
    imagePort.open("/visualizerImage");

    std::vector<iDynTree::PixelViz> pixels;

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastSent = std::chrono::steady_clock::now();

    unsigned int desiredFPS = 30;
    bool mirrorImage = false;

    long minimumMicroSec = std::round(1e6 / (double) desiredFPS);

    iDynTree::Transform wHb = iDynTree::Transform::Identity();
    iDynTree::VectorDynSize joints(loadedModel.getNrOfPosCoords());
    joints.zero();
    viz.modelViz("robot").setPositions(wHb, joints);

    while(viz.run())
    {
        viz.draw();
        now = std::chrono::steady_clock::now();

        if (std::chrono::duration_cast<std::chrono::microseconds>(now - lastSent).count() >= minimumMicroSec)
        {

            if (viz.getTexturePixels(pixels))
            {
                for (unsigned int i = 0; i < pixels.size(); ++i)
                {
                    iDynTree::PixelViz& pixelImage = pixels[i];

                    size_t width;
                    if (mirrorImage)
                    {
                        width = image.width() - 1 - pixelImage.width;
                    }
                    else
                    {
                        width = pixelImage.width;
                    }

                    yarp::sig::PixelRgba& pixelYarp = *(reinterpret_cast<yarp::sig::PixelRgba*>(
                                                            image.getPixelAddress(width, pixelImage.height)));

                    pixelYarp.r = pixelImage.r;
                    pixelYarp.g = pixelImage.g;
                    pixelYarp.b = pixelImage.b;
                    pixelYarp.a = pixelImage.a;

                }
            }
            yarp::sig::FlexImage& imageToBeSent = imagePort.prepare();
            imageToBeSent.setPixelCode(VOCAB_PIXEL_RGBA);
            imageToBeSent.setExternal(image.getRawImage(), image.width(), image.height()); //Avoid to copy
            imagePort.write();
            lastSent = std::chrono::steady_clock::now();
        }
    }

    image.resize(0,0);
    imagePort.close();

    return 0;
}
