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
#include <QProcessEnvironment>
#include <QDir>
#include <QFile>
#include <QFileInfo>


int main()
{
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    QString gazeboModelPath = env.value("GAZEBO_MODEL_PATH");
    QStringList paths = gazeboModelPath.split(":");
    QDir::setSearchPaths("meshPrefix", paths);

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
                QString meshPath = QString::fromStdString(originalPath);
                meshPath.replace("package://", "meshPrefix:");
                QFile meshFile(meshPath);
                QFileInfo meshFileInfo(meshFile);
                std::string meshFileInfoPath = meshFileInfo.absoluteFilePath().toStdString();
                mesh->setFilename(meshFileInfoPath);
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
