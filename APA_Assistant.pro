#-------------------------------------------------
#
# Project created by QtCreator 2019-08-30T14:39:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = APA_Assistant
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
CONFIG += resources_big
CONFIG += precompile_header
PRECOMPILED_HEADER = stable.h

SOURCES += \
        Common/Configure/Configs/vehilce_config.cpp \
        Common/Filter/digital_filter.cpp \
        Common/Filter/digital_filter_coefficients.cpp \
        Common/Math/algebraic_geometry.cpp \
        Common/Math/crc_compute.cpp \
        Common/Math/curve_fitting.cpp \
        Common/Math/fresnel.cpp \
        Common/Math/huogh.cpp \
        Common/Math/interpolation.cpp \
        Common/Math/linear_quadratic_regulator.cpp \
        Common/Math/math_utils.cpp \
        Common/Math/solve_equation.cpp \
        Common/Math/vector_2d.cpp \
        Common/Math/vehicle_body.cpp \
        Common/Utils/Src/link_list.cpp \
        Common/Utils/Src/node.cpp \
        Common/VehicleState/GeometricTrack/geometric_track.cpp \
        Common/VehicleState/Interface/vehicle_state.cpp \
        Control/Common/pid.cpp \
        Control/Common/trajectory_analyzer.cpp \
        Control/Interface/controller.cpp \
        Control/LatControl/lat_control.cpp \
        Control/LatControl/lat_control_lqr.cpp \
        Control/LonControl/lon_control.cpp \
        Interaction/CANBUS/BoRui/bo_rui_controller.cpp \
        Interaction/CANBUS/BoRui/bo_rui_message.cpp \
        Interaction/CANBUS/ChangAn/chang_an_controller.cpp \
        Interaction/CANBUS/ChangAn/chang_an_message.cpp \
        Interaction/CANBUS/DongFengE70/dong_feng_e70_message.cpp \
        Interaction/CANBUS/Interface/message_manager.cpp \
        Interaction/CANBUS/Interface/vehicle_controller.cpp \
        Interaction/HMI/Terminal.cpp \
        Interaction/HMI/simulation.cpp \
        Interaction/Ultrasonic/Ultrasonic.cpp \
        Percaption/Interface/percaption.cpp \
        Percaption/UltrasonicPercaption/ultrasonic_obstacle_percption.cpp \
        Planning/Common/configuration.cpp \
        Planning/Common/steering_common.cpp \
        Planning/Curvature/curvature.cpp \
        Planning/HC_CC_StateSpace/hc00_reeds_shepp_state_space.cpp \
        Planning/HC_CC_StateSpace/hc0pm_reeds_shepp_state_space.cpp \
        Planning/HC_CC_StateSpace/hc_reeds_shepp_state_space.cpp \
        Planning/HC_CC_StateSpace/hcpm0_reeds_shepp_state_space.cpp \
        Planning/HC_CC_StateSpace/hcpmpm_reeds_shepp_state_space.cpp \
        Planning/Interface/hc_cc_state_space.cpp \
        Planning/Interface/planning.cpp \
        Planning/ParallelParking/parallel_planning.cpp \
        Planning/Path/hc_cc_circle.cpp \
        Planning/Path/hc_cc_rs_path.cpp \
        Planning/Path/path.cpp \
        Planning/Path/path_line.cpp \
        Planning/VerticalParking/vertical_planning.cpp \
        QCustomPlot/axistag.cpp \
        main.cpp \
        mainwindow.cpp \
        QCustomPlot/qcustomplot.cpp \
#        ompl/base/goals/src/GoalLazySamples.cpp \
#        ompl/base/goals/src/GoalRegion.cpp \
#        ompl/base/goals/src/GoalState.cpp \
#        ompl/base/goals/src/GoalStates.cpp \
#        ompl/base/objectives/src/MaximizeMinClearanceObjective.cpp \
#        ompl/base/objectives/src/MechanicalWorkOptimizationObjective.cpp \
#        ompl/base/objectives/src/MinimaxObjective.cpp \
#        ompl/base/objectives/src/PathLengthOptimizationObjective.cpp \
#        ompl/base/objectives/src/StateCostIntegralObjective.cpp \
#        ompl/base/samplers/informed/src/OrderedInfSampler.cpp \
#        ompl/base/samplers/informed/src/PathLengthDirectInfSampler.cpp \
#        ompl/base/samplers/informed/src/RejectionInfSampler.cpp \
#        ompl/base/samplers/src/BridgeTestValidStateSampler.cpp \
#        ompl/base/samplers/src/GaussianValidStateSampler.cpp \
#        ompl/base/samplers/src/InformedStateSampler.cpp \
#        ompl/base/samplers/src/MaximizeClearanceValidStateSampler.cpp \
#        ompl/base/samplers/src/MinimumClearanceValidStateSampler.cpp \
#        ompl/base/samplers/src/ObstacleBasedValidStateSampler.cpp \
#        ompl/base/samplers/src/UniformValidStateSampler.cpp \
#        ompl/base/spaces/constraint/src/AtlasChart.cpp \
#        ompl/base/spaces/constraint/src/AtlasStateSpace.cpp \
#        ompl/base/spaces/constraint/src/ConstrainedStateSpace.cpp \
#        ompl/base/spaces/constraint/src/ProjectedStateSpace.cpp \
#        ompl/base/spaces/constraint/src/TangentBundleStateSpace.cpp \
#        ompl/base/spaces/src/DiscreteStateSpace.cpp \
#        ompl/base/spaces/src/DubinsStateSpace.cpp \
#        ompl/base/spaces/src/RealVectorBounds.cpp \
#        ompl/base/spaces/src/RealVectorStateProjections.cpp \
#        ompl/base/spaces/src/RealVectorStateSpace.cpp \
#        ompl/base/spaces/src/ReedsSheppStateSpace.cpp \
#        ompl/base/spaces/src/SE2StateSpace.cpp \
#        ompl/base/spaces/src/SE3StateSpace.cpp \
#        ompl/base/spaces/src/SO2StateSpace.cpp \
#        ompl/base/spaces/src/SO3StateSpace.cpp \
#        ompl/base/spaces/src/TimeStateSpace.cpp \
#        ompl/base/spaces/src/WrapperStateSpace.cpp \
#        ompl/base/src/Constraint.cpp \
#        ompl/base/src/Cost.cpp \
#        ompl/base/src/DiscreteMotionValidator.cpp \
#        ompl/base/src/GenericParam.cpp \
#        ompl/base/src/Goal.cpp \
#        ompl/base/src/OptimizationObjective.cpp \
#        ompl/base/src/Planner.cpp \
#        ompl/base/src/PlannerData.cpp \
#        ompl/base/src/PlannerDataStorage.cpp \
#        ompl/base/src/PlannerStatus.cpp \
#        ompl/base/src/PlannerTerminationCondition.cpp \
#        ompl/base/src/PrecomputedStateSampler.cpp \
#        ompl/base/src/ProblemDefinition.cpp \
#        ompl/base/src/ProjectionEvaluator.cpp \
#        ompl/base/src/SpaceInformation.cpp \
#        ompl/base/src/StateSampler.cpp \
#        ompl/base/src/StateSpace.cpp \
#        ompl/base/src/StateStorage.cpp \
#        ompl/base/src/ValidStateSampler.cpp \
#        ompl/control/planners/est/src/EST.cpp \
#        ompl/control/planners/kpiece/src/KPIECE1.cpp \
#        ompl/control/planners/ltl/src/Automaton.cpp \
#        ompl/control/planners/ltl/src/LTLPlanner.cpp \
#        ompl/control/planners/ltl/src/LTLProblemDefinition.cpp \
#        ompl/control/planners/ltl/src/LTLSpaceInformation.cpp \
#        ompl/control/planners/ltl/src/ProductGraph.cpp \
#        ompl/control/planners/ltl/src/PropositionalDecomposition.cpp \
#        ompl/control/planners/ltl/src/World.cpp \
#        ompl/control/planners/pdst/src/PDST.cpp \
#        ompl/control/planners/rrt/src/RRT.cpp \
#        ompl/control/planners/sst/src/SST.cpp \
#        ompl/control/planners/syclop/src/GridDecomposition.cpp \
#        ompl/control/planners/syclop/src/Syclop.cpp \
#        ompl/control/planners/syclop/src/SyclopEST.cpp \
#        ompl/control/planners/syclop/src/SyclopRRT.cpp \
#        ompl/control/spaces/src/DiscreteControlSpace.cpp \
#        ompl/control/spaces/src/RealVectorControlSpace.cpp \
#        ompl/control/src/ControlSampler.cpp \
#        ompl/control/src/ControlSpace.cpp \
#        ompl/control/src/PathControl.cpp \
#        ompl/control/src/PlannerData.cpp \
#        ompl/control/src/PlannerDataStorage.cpp \
#        ompl/control/src/SimpleDirectedControlSampler.cpp \
#        ompl/control/src/SimpleSetup.cpp \
#        ompl/control/src/SpaceInformation.cpp \
#        ompl/extensions/morse/src/MorseControlSpace.cpp \
#        ompl/extensions/morse/src/MorseEnvironment.cpp \
#        ompl/extensions/morse/src/MorseGoal.cpp \
#        ompl/extensions/morse/src/MorseProjection.cpp \
#        ompl/extensions/morse/src/MorseSimpleSetup.cpp \
#        ompl/extensions/morse/src/MorseStatePropagator.cpp \
#        ompl/extensions/morse/src/MorseStateSpace.cpp \
#        ompl/extensions/morse/src/MorseStateValidityChecker.cpp \
#        ompl/extensions/opende/src/OpenDEControlSpace.cpp \
#        ompl/extensions/opende/src/OpenDEEnvironment.cpp \
#        ompl/extensions/opende/src/OpenDESimpleSetup.cpp \
#        ompl/extensions/opende/src/OpenDEStatePropagator.cpp \
#        ompl/extensions/opende/src/OpenDEStateSpace.cpp \
#        ompl/extensions/opende/src/OpenDEStateValidityChecker.cpp \
#        ompl/extensions/triangle/src/PropositionalTriangularDecomposition.cpp \
#        ompl/extensions/triangle/src/TriangularDecomposition.cpp \
#        ompl/geometric/planners/AnytimePathShortening.cpp \
#        ompl/geometric/planners/bitstar/datastructures/src/ImplicitGraph.cpp \
#        ompl/geometric/planners/bitstar/datastructures/src/SearchQueue.cpp \
#        ompl/geometric/planners/bitstar/datastructures/src/Vertex.cpp \
#        ompl/geometric/planners/bitstar/src/BITstar.cpp \
#        ompl/geometric/planners/cforest/src/CForest.cpp \
#        ompl/geometric/planners/cforest/src/CForestStateSampler.cpp \
#        ompl/geometric/planners/cforest/src/CForestStateSpaceWrapper.cpp \
#        ompl/geometric/planners/est/src/BiEST.cpp \
#        ompl/geometric/planners/est/src/EST.cpp \
#        ompl/geometric/planners/est/src/ProjEST.cpp \
#        ompl/geometric/planners/experience/src/LightningRetrieveRepair.cpp \
#        ompl/geometric/planners/experience/src/ThunderRetrieveRepair.cpp \
#        ompl/geometric/planners/fmt/src/BFMT.cpp \
#        ompl/geometric/planners/fmt/src/FMT.cpp \
#        ompl/geometric/planners/kpiece/src/BKPIECE1.cpp \
#        ompl/geometric/planners/kpiece/src/KPIECE1.cpp \
#        ompl/geometric/planners/kpiece/src/LBKPIECE1.cpp \
#        ompl/geometric/planners/pdst/src/PDST.cpp \
#        ompl/geometric/planners/prm/src/LazyPRM.cpp \
#        ompl/geometric/planners/prm/src/LazyPRMstar.cpp \
#        ompl/geometric/planners/prm/src/PRM.cpp \
#        ompl/geometric/planners/prm/src/PRMstar.cpp \
#        ompl/geometric/planners/prm/src/SPARS.cpp \
#        ompl/geometric/planners/prm/src/SPARStwo.cpp \
#        ompl/geometric/planners/rrt/src/BiTRRT.cpp \
#        ompl/geometric/planners/rrt/src/InformedRRTstar.cpp \
#        ompl/geometric/planners/rrt/src/LBTRRT.cpp \
#        ompl/geometric/planners/rrt/src/LazyLBTRRT.cpp \
#        ompl/geometric/planners/rrt/src/LazyRRT.cpp \
#        ompl/geometric/planners/rrt/src/RRT.cpp \
#        ompl/geometric/planners/rrt/src/RRTConnect.cpp \
#        ompl/geometric/planners/rrt/src/RRTXstatic.cpp \
#        ompl/geometric/planners/rrt/src/RRTsharp.cpp \
#        ompl/geometric/planners/rrt/src/RRTstar.cpp \
#        ompl/geometric/planners/rrt/src/SORRTstar.cpp \
#        ompl/geometric/planners/rrt/src/TRRT.cpp \
#        ompl/geometric/planners/rrt/src/VFRRT.cpp \
#        ompl/geometric/planners/rrt/src/pRRT.cpp \
#        ompl/geometric/planners/sbl/src/SBL.cpp \
#        ompl/geometric/planners/sbl/src/pSBL.cpp \
#        ompl/geometric/planners/sst/src/SST.cpp \
#        ompl/geometric/planners/stride/src/STRIDE.cpp \
#        ompl/geometric/src/GeneticSearch.cpp \
#        ompl/geometric/src/HillClimbing.cpp \
#        ompl/geometric/src/PathGeometric.cpp \
#        ompl/geometric/src/PathHybridization.cpp \
#        ompl/geometric/src/PathSimplifier.cpp \
#        ompl/geometric/src/SimpleSetup.cpp \
#        ompl/tools/benchmark/src/Benchmark.cpp \
#        ompl/tools/benchmark/src/MachineSpecs.cpp \
#        ompl/tools/config/src/SelfConfig.cpp \
#        ompl/tools/debug/src/PlannerMonitor.cpp \
#        ompl/tools/debug/src/Profiler.cpp \
#        ompl/tools/experience/src/ExperienceSetup.cpp \
#        ompl/tools/lightning/src/DynamicTimeWarp.cpp \
#        ompl/tools/lightning/src/Lightning.cpp \
#        ompl/tools/lightning/src/LightningDB.cpp \
#        ompl/tools/multiplan/src/OptimizePlan.cpp \
#        ompl/tools/multiplan/src/ParallelPlan.cpp \
#        ompl/tools/thunder/src/SPARSdb.cpp \
#        ompl/tools/thunder/src/Thunder.cpp \
#        ompl/tools/thunder/src/ThunderDB.cpp \
#        ompl/util/src/Console.cpp \
#        ompl/util/src/GeometricEquations.cpp \
#        ompl/util/src/PPM.cpp \
#        ompl/util/src/ProlateHyperspheroid.cpp \
#        ompl/util/src/RandomNumbers.cpp \
#        ompl/util/src/String.cpp

HEADERS += \
        Common/Configure/Configs/system_config.h \
        Common/Configure/Configs/vehilce_config.h \
        Common/Configure/Data/bo_rui_configure.h \
        Common/Configure/Data/chang_an_configure.h \
        Common/Configure/Data/common_configure.h \
        Common/Configure/Data/dong_feng_configure.h \
        Common/Filter/digital_filter.h \
        Common/Filter/digital_filter_coefficients.h \
        Common/Math/algebraic_geometry.h \
        Common/Math/crc_compute.h \
        Common/Math/curve_fitting.h \
        Common/Math/fresnel.h \
        Common/Math/huogh.h \
        Common/Math/interpolation.h \
        Common/Math/linear_quadratic_regulator.h \
        Common/Math/math_utils.h \
        Common/Math/solve_equation.h \
        Common/Math/vector_2d.h \
        Common/Math/vehicle_body.h \
        Common/Utils/Inc/link_list.h \
        Common/Utils/Inc/node.h \
        Common/Utils/Inc/property.h \
        Common/VehicleState/GeometricTrack/geometric_track.h \
        Common/VehicleState/Interface/vehicle_state.h \
        Control/Common/pid.h \
        Control/Common/trajectory_analyzer.h \
        Control/Interface/controller.h \
        Control/LatControl/lat_control.h \
        Control/LatControl/lat_control_lqr.h \
        Control/LonControl/lon_control.h \
        Interaction/CANBUS/BoRui/bo_rui_controller.h \
        Interaction/CANBUS/BoRui/bo_rui_message.h \
        Interaction/CANBUS/ChangAn/chang_an_controller.h \
        Interaction/CANBUS/ChangAn/chang_an_message.h \
        Interaction/CANBUS/DongFengE70/dong_feng_e70_controller.h \
        Interaction/CANBUS/DongFengE70/dong_feng_e70_message.h \
        Interaction/CANBUS/Interface/message_manager.h \
        Interaction/CANBUS/Interface/vehicle_controller.h \
        Interaction/HMI/Terminal.h \
        Interaction/HMI/simulation.h \
        Interaction/Ultrasonic/Ultrasonic.h \
        Percaption/Interface/percaption.h \
        Percaption/UltrasonicPercaption/ultrasonic_obstacle_percption.h \
        Planning/Common/configuration.h \
        Planning/Common/steering_common.h \
        Planning/Curvature/curvature.h \
        Planning/HC_CC_StateSpace/hc00_reeds_shepp_state_space.h \
        Planning/HC_CC_StateSpace/hc0pm_reeds_shepp_state_space.h \
        Planning/HC_CC_StateSpace/hc_reeds_shepp_state_space.h \
        Planning/HC_CC_StateSpace/hcpm0_reeds_shepp_state_space.h \
        Planning/HC_CC_StateSpace/hcpmpm_reeds_shepp_state_space.h \
        Planning/Interface/hc_cc_state_space.h \
        Planning/Interface/planning.h \
        Planning/ParallelParking/parallel_planning.h \
        Planning/Path/hc_cc_circle.h \
        Planning/Path/hc_cc_rs_path.h \
        Planning/Path/path.h \
        Planning/Path/path_line.h \
        Planning/VerticalParking/vertical_planning.h \
        QCustomPlot/axistag.h \
        mainwindow.h \
        stable.h \
        QCustomPlot/qcustomplot.h \ \
#        ompl/base/ConstrainedSpaceInformation.h \
#        ompl/base/Constraint.h \
#        ompl/base/Cost.h \
#        ompl/base/DiscreteMotionValidator.h \
#        ompl/base/GenericParam.h \
#        ompl/base/Goal.h \
#        ompl/base/GoalTypes.h \
#        ompl/base/MotionValidator.h \
#        ompl/base/OptimizationObjective.h \
#        ompl/base/Path.h \
#        ompl/base/Planner.h \
#        ompl/base/PlannerData.h \
#        ompl/base/PlannerDataGraph.h \
#        ompl/base/PlannerDataStorage.h \
#        ompl/base/PlannerStatus.h \
#        ompl/base/PlannerTerminationCondition.h \
#        ompl/base/PrecomputedStateSampler.h \
#        ompl/base/ProblemDefinition.h \
#        ompl/base/ProjectionEvaluator.h \
#        ompl/base/ScopedState.h \
#        ompl/base/SolutionNonExistenceProof.h \
#        ompl/base/SpaceInformation.h \
#        ompl/base/State.h \
#        ompl/base/StateSampler.h \
#        ompl/base/StateSamplerArray.h \
#        ompl/base/StateSpace.h \
#        ompl/base/StateSpaceTypes.h \
#        ompl/base/StateStorage.h \
#        ompl/base/StateValidityChecker.h \
#        ompl/base/TypedSpaceInformation.h \
#        ompl/base/TypedStateValidityChecker.h \
#        ompl/base/ValidStateSampler.h \
#        ompl/base/goals/GoalLazySamples.h \
#        ompl/base/goals/GoalRegion.h \
#        ompl/base/goals/GoalSampleableRegion.h \
#        ompl/base/goals/GoalState.h \
#        ompl/base/goals/GoalStates.h \
#        ompl/base/objectives/MaximizeMinClearanceObjective.h \
#        ompl/base/objectives/MechanicalWorkOptimizationObjective.h \
#        ompl/base/objectives/MinimaxObjective.h \
#        ompl/base/objectives/PathLengthOptimizationObjective.h \
#        ompl/base/objectives/StateCostIntegralObjective.h \
#        ompl/base/objectives/VFMechanicalWorkOptimizationObjective.h \
#        ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.h \
#        ompl/base/samplers/BridgeTestValidStateSampler.h \
#        ompl/base/samplers/GaussianValidStateSampler.h \
#        ompl/base/samplers/InformedStateSampler.h \
#        ompl/base/samplers/MaximizeClearanceValidStateSampler.h \
#        ompl/base/samplers/MinimumClearanceValidStateSampler.h \
#        ompl/base/samplers/ObstacleBasedValidStateSampler.h \
#        ompl/base/samplers/UniformValidStateSampler.h \
#        ompl/base/samplers/informed/OrderedInfSampler.h \
#        ompl/base/samplers/informed/PathLengthDirectInfSampler.h \
#        ompl/base/samplers/informed/RejectionInfSampler.h \
#        ompl/base/spaces/DiscreteStateSpace.h \
#        ompl/base/spaces/DubinsStateSpace.h \
#        ompl/base/spaces/RealVectorBounds.h \
#        ompl/base/spaces/RealVectorStateProjections.h \
#        ompl/base/spaces/RealVectorStateSpace.h \
#        ompl/base/spaces/ReedsSheppStateSpace.h \
#        ompl/base/spaces/SE2StateSpace.h \
#        ompl/base/spaces/SE3StateSpace.h \
#        ompl/base/spaces/SO2StateSpace.h \
#        ompl/base/spaces/SO3StateSpace.h \
#        ompl/base/spaces/TimeStateSpace.h \
#        ompl/base/spaces/WrapperStateSpace.h \
#        ompl/base/spaces/constraint/AtlasChart.h \
#        ompl/base/spaces/constraint/AtlasStateSpace.h \
#        ompl/base/spaces/constraint/ConstrainedStateSpace.h \
#        ompl/base/spaces/constraint/ProjectedStateSpace.h \
#        ompl/base/spaces/constraint/TangentBundleStateSpace.h \
#        ompl/config.h \
#        ompl/control/Control.h \
#        ompl/control/ControlSampler.h \
#        ompl/control/ControlSpace.h \
#        ompl/control/ControlSpaceTypes.h \
#        ompl/control/DirectedControlSampler.h \
#        ompl/control/ODESolver.h \
#        ompl/control/PathControl.h \
#        ompl/control/PlannerData.h \
#        ompl/control/PlannerDataStorage.h \
#        ompl/control/SimpleDirectedControlSampler.h \
#        ompl/control/SimpleSetup.h \
#        ompl/control/SpaceInformation.h \
#        ompl/control/StatePropagator.h \
#        ompl/control/SteeredControlSampler.h \
#        ompl/control/planners/PlannerIncludes.h \
#        ompl/control/planners/est/EST.h \
#        ompl/control/planners/kpiece/KPIECE1.h \
#        ompl/control/planners/ltl/Automaton.h \
#        ompl/control/planners/ltl/LTLPlanner.h \
#        ompl/control/planners/ltl/LTLProblemDefinition.h \
#        ompl/control/planners/ltl/LTLSpaceInformation.h \
#        ompl/control/planners/ltl/ProductGraph.h \
#        ompl/control/planners/ltl/PropositionalDecomposition.h \
#        ompl/control/planners/ltl/World.h \
#        ompl/control/planners/pdst/PDST.h \
#        ompl/control/planners/rrt/RRT.h \
#        ompl/control/planners/sst/SST.h \
#        ompl/control/planners/syclop/Decomposition.h \
#        ompl/control/planners/syclop/GridDecomposition.h \
#        ompl/control/planners/syclop/Syclop.h \
#        ompl/control/planners/syclop/SyclopEST.h \
#        ompl/control/planners/syclop/SyclopRRT.h \
#        ompl/control/spaces/DiscreteControlSpace.h \
#        ompl/control/spaces/RealVectorControlSpace.h \
#        ompl/datastructures/BinaryHeap.h \
#        ompl/datastructures/DynamicSSSP.h \
#        ompl/datastructures/GreedyKCenters.h \
#        ompl/datastructures/Grid.h \
#        ompl/datastructures/GridB.h \
#        ompl/datastructures/GridN.h \
#        ompl/datastructures/LPAstarOnGraph.h \
#        ompl/datastructures/NearestNeighbors.h \
#        ompl/datastructures/NearestNeighborsFLANN.h \
#        ompl/datastructures/NearestNeighborsGNAT.h \
#        ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h \
#        ompl/datastructures/NearestNeighborsLinear.h \
#        ompl/datastructures/NearestNeighborsSqrtApprox.h \
#        ompl/datastructures/PDF.h \
#        ompl/datastructures/Permutation.h \
#        ompl/extensions/morse/MorseControlSpace.h \
#        ompl/extensions/morse/MorseEnvironment.h \
#        ompl/extensions/morse/MorseGoal.h \
#        ompl/extensions/morse/MorseProjection.h \
#        ompl/extensions/morse/MorseSimpleSetup.h \
#        ompl/extensions/morse/MorseStatePropagator.h \
#        ompl/extensions/morse/MorseStateSpace.h \
#        ompl/extensions/morse/MorseStateValidityChecker.h \
#        ompl/extensions/morse/MorseTerminationCondition.h \
#        ompl/extensions/opende/OpenDEControlSpace.h \
#        ompl/extensions/opende/OpenDEEnvironment.h \
#        ompl/extensions/opende/OpenDESimpleSetup.h \
#        ompl/extensions/opende/OpenDEStatePropagator.h \
#        ompl/extensions/opende/OpenDEStateSpace.h \
#        ompl/extensions/opende/OpenDEStateValidityChecker.h \
#        ompl/extensions/triangle/PropositionalTriangularDecomposition.h \
#        ompl/extensions/triangle/TriangularDecomposition.h \
#        ompl/geometric/GeneticSearch.h \
#        ompl/geometric/HillClimbing.h \
#        ompl/geometric/PathGeometric.h \
#        ompl/geometric/PathHybridization.h \
#        ompl/geometric/PathSimplifier.h \
#        ompl/geometric/SimpleSetup.h \
#        ompl/geometric/planners/AnytimePathShortening.h \
#        ompl/geometric/planners/PlannerIncludes.h \
#        ompl/geometric/planners/bitstar/BITstar.h \
#        ompl/geometric/planners/bitstar/datastructures/CostHelper.h \
#        ompl/geometric/planners/bitstar/datastructures/HelperFunctions.h \
#        ompl/geometric/planners/bitstar/datastructures/IdGenerator.h \
#        ompl/geometric/planners/bitstar/datastructures/ImplicitGraph.h \
#        ompl/geometric/planners/bitstar/datastructures/SearchQueue.h \
#        ompl/geometric/planners/bitstar/datastructures/Vertex.h \
#        ompl/geometric/planners/cforest/CForest.h \
#        ompl/geometric/planners/cforest/CForestStateSampler.h \
#        ompl/geometric/planners/cforest/CForestStateSpaceWrapper.h \
#        ompl/geometric/planners/est/BiEST.h \
#        ompl/geometric/planners/est/EST.h \
#        ompl/geometric/planners/est/ProjEST.h \
#        ompl/geometric/planners/experience/LightningRetrieveRepair.h \
#        ompl/geometric/planners/experience/ThunderRetrieveRepair.h \
#        ompl/geometric/planners/fmt/BFMT.h \
#        ompl/geometric/planners/fmt/FMT.h \
#        ompl/geometric/planners/kpiece/BKPIECE1.h \
#        ompl/geometric/planners/kpiece/Discretization.h \
#        ompl/geometric/planners/kpiece/KPIECE1.h \
#        ompl/geometric/planners/kpiece/LBKPIECE1.h \
#        ompl/geometric/planners/pdst/PDST.h \
#        ompl/geometric/planners/prm/ConnectionStrategy.h \
#        ompl/geometric/planners/prm/LazyPRM.h \
#        ompl/geometric/planners/prm/LazyPRMstar.h \
#        ompl/geometric/planners/prm/PRM.h \
#        ompl/geometric/planners/prm/PRMstar.h \
#        ompl/geometric/planners/prm/SPARS.h \
#        ompl/geometric/planners/prm/SPARStwo.h \
#        ompl/geometric/planners/prm/src/GoalVisitor.hpp \
#        ompl/geometric/planners/rrt/BiTRRT.h \
#        ompl/geometric/planners/rrt/InformedRRTstar.h \
#        ompl/geometric/planners/rrt/LBTRRT.h \
#        ompl/geometric/planners/rrt/LazyLBTRRT.h \
#        ompl/geometric/planners/rrt/LazyRRT.h \
#        ompl/geometric/planners/rrt/RRT.h \
#        ompl/geometric/planners/rrt/RRTConnect.h \
#        ompl/geometric/planners/rrt/RRTXstatic.h \
#        ompl/geometric/planners/rrt/RRTsharp.h \
#        ompl/geometric/planners/rrt/RRTstar.h \
#        ompl/geometric/planners/rrt/SORRTstar.h \
#        ompl/geometric/planners/rrt/TRRT.h \
#        ompl/geometric/planners/rrt/VFRRT.h \
#        ompl/geometric/planners/rrt/pRRT.h \
#        ompl/geometric/planners/sbl/SBL.h \
#        ompl/geometric/planners/sbl/pSBL.h \
#        ompl/geometric/planners/sst/SST.h \
#        ompl/geometric/planners/stride/STRIDE.h \
#        ompl/tools/benchmark/Benchmark.h \
#        ompl/tools/benchmark/MachineSpecs.h \
#        ompl/tools/config/MagicConstants.h \
#        ompl/tools/config/SelfConfig.h \
#        ompl/tools/debug/PlannerMonitor.h \
#        ompl/tools/debug/Profiler.h \
#        ompl/tools/experience/ExperienceSetup.h \
#        ompl/tools/lightning/DynamicTimeWarp.h \
#        ompl/tools/lightning/Lightning.h \
#        ompl/tools/lightning/LightningDB.h \
#        ompl/tools/multiplan/OptimizePlan.h \
#        ompl/tools/multiplan/ParallelPlan.h \
#        ompl/tools/thunder/SPARSdb.h \
#        ompl/tools/thunder/Thunder.h \
#        ompl/tools/thunder/ThunderDB.h \
#        ompl/util/ClassForward.h \
#        ompl/util/Console.h \
#        ompl/util/Deprecation.h \
#        ompl/util/DisableCompilerWarning.h \
#        ompl/util/Exception.h \
#        ompl/util/GeometricEquations.h \
#        ompl/util/Hash.h \
#        ompl/util/PPM.h \
#        ompl/util/ProlateHyperspheroid.h \
#        ompl/util/RandomNumbers.h \
#        ompl/util/String.h \
#        ompl/util/Time.h \


LIBS += -L$$PWD/WinZlgCan/ -lControlCAN

LIBS += "C:/Boost/lib/libboost_date_time-mgw7-mt-x64-1_73.a"
LIBS += "C:/Boost/lib/libboost_thread-mgw7-mt-x64-1_73.a"
LIBS += "C:/Boost/lib/libboost_serialization-mgw7-mt-x64-1_73.a"
LIBS += "C:/Boost/lib/libboost_filesystem-mgw7-mt-x64-1_73.a"
LIBS += "C:/Boost/lib/libboost_system-mgw7-mt-x64-1_73.a"
LIBS += "C:/Boost/lib/libboost_program_options-mgw7-mt-x64-1_73.a"
LIBS += "C:/Boost/lib/libboost_test_exec_monitor-mgw7-mt-x64-1_73.a"
LIBS += "C:/Boost/lib/libboost_chrono-mgw7-mt-x64-1_73.a"


INCLUDEPATH += C:\eigen-3.3.7
INCLUDEPATH += C:\Boost\include\boost-1_73

DEFINES += BOOST_USE_LIB

FORMS += \
        mainwindow.ui

RESOURCES += \
    icon.qrc

DISTFILES +=
