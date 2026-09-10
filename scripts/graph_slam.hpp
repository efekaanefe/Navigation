#pragma once

#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/linear/NoiseModel.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"

#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"

#include <eigen3/Eigen/Dense>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <vector>

#include "utils.hpp"

using namespace gtsam;

Edge_SE2 get_edge( const Data2D *data, int index ) { return data->edges[index]; };

Edge_SE3 get_edge( const Data3D *data, int index ) { return data->edges[index]; };

Pose2 get_pose( const Data2D *data, int index ) {
    const auto &vertex = data->vertices[index];
    return Pose2( vertex.x, vertex.y, vertex.theta );
};

Pose3 get_pose( const Data3D *data, int index ) {
    const auto &vertex = data->vertices[index];
    return Pose3( Rot3::Quaternion( vertex.qw, vertex.qx, vertex.qy, vertex.qz ),
                  Point3( vertex.x, vertex.y, vertex.z ) );
};

template <typename DataType>
void propogate_graph_onestep( NonlinearFactorGraph *graph, DataType *data, int curr_index ) {

    const auto curr_edge = get_edge( data, curr_index );
    const auto odometry_mean = get_pose( data, curr_index );

    auto cov_matrix = get_covariance_matrix( curr_edge );
    noiseModel::Diagonal::shared_ptr odometry_noise = noiseModel::Diagonal::Sigmas( cov_matrix.diagonal().cwiseSqrt() );

    if constexpr ( std::is_same<DataType, Data3D>::value )
        graph->add( BetweenFactor<Pose3>( curr_edge.indeces[0], curr_edge.indeces[1], odometry_mean, odometry_noise ) );
    else if constexpr ( std::is_same<DataType, Data2D>::value )
        graph->add( BetweenFactor<Pose2>( curr_edge.indeces[0], curr_edge.indeces[1], odometry_mean, odometry_noise ) );
};

template <typename PoseType> void add_prior_factor( NonlinearFactorGraph *graph, PoseType priorMean ) {
    noiseModel::Diagonal::shared_ptr priorNoise;

    if ( std::is_same<PoseType, Pose2>::value ) {
        priorNoise = noiseModel::Diagonal::Sigmas( Vector3( 0.3, 0.3, 0.1 ) );

    } else if ( std::is_same<PoseType, Pose3>::value ) {
        priorNoise = noiseModel::Diagonal::Sigmas( Vector6( 0.3, 0.3, 0.3, 0.1, 0.1, 0.1 ) );
    }

    graph->add( PriorFactor<PoseType>( 0, priorMean, priorNoise ) );
};

template <typename DataType> NonlinearFactorGraph construct_graph( DataType *data ) {
    NonlinearFactorGraph graph;

    // prior
    if constexpr ( std::is_same<DataType, Data2D>::value ) {
        Pose2 priorMean = get_pose( data, 0 );
        add_prior_factor( &graph, priorMean );
    } else if constexpr ( std::is_same<DataType, Data3D>::value ) {
        Pose3 priorMean = get_pose( data, 0 );
        add_prior_factor( &graph, priorMean );
    }

    // adding one step for each measurements/edges
    for ( int curr_index = 0; curr_index < data->edges.size(); curr_index++ ) {
        propogate_graph_onestep( &graph, data, curr_index );
    }

    graph.print();

    return graph;
};

template <typename DataType> Values get_initial_guess( const DataType *data ) {
    Values initial;
    for ( const auto &vertex : data->vertices ) {
        if constexpr ( std::is_same<DataType, Data2D>::value )
            initial.insert( vertex.index, gtsam::Pose2( vertex.x, vertex.y, vertex.theta ) );
        else if constexpr ( std::is_same<DataType, Data3D>::value ) {
            initial.insert( vertex.index,
                            gtsam::Pose3( gtsam::Rot3::Quaternion( vertex.qw, vertex.qx, vertex.qy, vertex.qz ),
                                          gtsam::Point3( vertex.x, vertex.y, vertex.z ) ) );
        }
    }

    return initial;
};

template <typename DataType> auto construct_optimized_traj( Values *result ) {
    decltype( DataType::vertices ) trajectory;

    for ( const auto &key_value : *result ) {
        Key key = key_value.key;

        if constexpr ( std::is_same<DataType, Data3D>::value ) {
            Pose3 pose = key_value.value.cast<Pose3>();

            Vertex_SE3 vertex;
            vertex.index = key;
            vertex.x = pose.x();
            vertex.y = pose.y();
            vertex.z = pose.z();

            gtsam::Quaternion quat = pose.rotation().toQuaternion();
            vertex.qw = quat.w();
            vertex.qx = quat.x();
            vertex.qy = quat.y();
            vertex.qz = quat.z();

            trajectory.push_back( vertex );
        } else if constexpr ( std::is_same<DataType, Data2D>::value ) {

            Pose2 pose = key_value.value.cast<Pose2>();

            Vertex_SE2 vertex;
            vertex.index = key;
            vertex.x = pose.x();
            vertex.y = pose.y();
            vertex.theta = pose.theta();

            trajectory.push_back( vertex );
        }
    }

    return trajectory;
}

template <typename DataType> auto solve_slam( DataType *data, bool use_incremental = false ) {
    if ( !use_incremental ) {

        NonlinearFactorGraph graph = construct_graph( data );
        Values initial = get_initial_guess( data );

        Values result = LevenbergMarquardtOptimizer( graph, initial ).optimize();
        return construct_optimized_traj<DataType>( &result );

    } else {

        ISAM2 isam;
        Values initial;

        for ( int i = 0; i < data->vertices.size(); i++ ) {
            NonlinearFactorGraph new_graph;
            Values new_initial;

            auto curr_vertex = data->vertices[i];
            Key vertex_key = curr_vertex.index;

            if ( i == 0 ) {
                if constexpr ( std::is_same<DataType, Data2D>::value ) {
                    Pose2 priorMean = get_pose( data, 0 );
                    add_prior_factor( &new_graph, priorMean );
                } else if constexpr ( std::is_same<DataType, Data3D>::value ) {
                    Pose3 priorMean = get_pose( data, 0 );
                    add_prior_factor( &new_graph, priorMean );
                }
            }

            auto initial_pose = get_pose( data, i );

            new_initial.insert( vertex_key, initial_pose );

            for ( int j = 0; j < data->edges.size(); j++ ) {
                auto curr_edge = data->edges[j];

                int u = curr_edge.indeces[0];
                int v = curr_edge.indeces[1];
                if ( std::max( u, v ) == vertex_key ) {
                    propogate_graph_onestep( &new_graph, data, j );
                }
            }

            isam.update( new_graph, new_initial );
            initial = isam.calculateEstimate();
        }

        return construct_optimized_traj<DataType>( &initial );
    }
};
