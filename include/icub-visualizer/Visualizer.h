/*
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#ifndef ICUB_VISUALIZER_H
#define ICUB_VISUALIZER_H

#include <DynamicalPlanner/State.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Position.h>
#include <memory>

class Visualizer {

public:

    Visualizer();

    ~Visualizer();

    bool setModel(const iDynTree::Model& model);

    bool visualizeState(const DynamicalPlanner::State& stateToVisualize);

    bool visualizeStates(const std::vector<State>& states, double endTime = -1.0);

    bool visualizeStates(const std::vector<State>& states,
                         const std::vector<iDynTree::Position>& cameraPosition,
                         const std::vector<iDynTree::Position>& cameraTarget,
                         double endTime = -1.0);

    bool visualizeStatesAndSaveAnimation(const std::vector<State>& states, const std::string& workingFolder,
                                const std::string& fileName, const std::string& fileExtension = "gif", double endTime = -1.0);

    bool setCameraPosition(const iDynTree::Position& cameraPosition);

    bool setCameraTarget(const iDynTree::Position& cameraTarget);

    bool setLightDirection(const iDynTree::Direction& lightDirection);

private:

    class VisualizerImplementation;
    std::unique_ptr<VisualizerImplementation> m_pimpl;
};

#endif // ICUB_VISUALIZER_H
