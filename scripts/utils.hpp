#pragma once

#include <boost/exception/exception.hpp>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct Vertex_SE2 {
    int index;
    double x, y, theta;
};
struct Vertex_XY;

struct Edge_SE2 {
    int indeces[2];
    double x, y, theta;
    double info[6];
};

struct Data2D {
    std::vector<Vertex_SE2> vertices;
    std::vector<Edge_SE2> edges;
};

struct Vertex_SE3 {
    int index;
    double x, y, z, qx, qy, qz, qw;
};

struct Edge_SE3 {
    int indeces[2];
    double x, y, z, qx, qy, qz, qw;
    double info[21];
};

struct Data3D {
    std::vector<Vertex_SE3> vertices;
    std::vector<Edge_SE3> edges;
};

// Load
Data2D read_data_2D( std::string filepath ) {
    Data2D data;
    std::ifstream file( filepath );

    if ( !file.is_open() ) {
        std::cerr << "Error while reading data\n";
        return data;
    }

    std::string line;
    while ( std::getline( file, line ) ) {
        std::istringstream iss( line );

        std::string type;
        iss >> type;

        if ( type == "VERTEX_SE2" ) {
            int index;
            double x, y, theta;
            iss >> index >> x >> y >> theta;

            Vertex_SE2 vertex{ .index = index, .x = x, .y = y, .theta = theta };
            data.vertices.push_back( vertex );
        }

        // TODO: check if indeces are sequential to flag loop closures
        else if ( type == "EDGE_SE2" ) {
            int indeces[2];
            double x, y, theta;
            double info[6];

            iss >> indeces[0] >> indeces[1] >> x >> y >> theta >> info[0] >> info[1] >> info[2] >> info[3] >> info[4] >>
                info[5];

            Edge_SE2 edge{ .x = x, .y = y, .theta = theta };
            edge.indeces[0] = indeces[0];
            edge.indeces[1] = indeces[1];
            for ( int i = 0; i < 6; ++i ) {
                edge.info[i] = info[i];
            }

            data.edges.push_back( edge );
        }
    }

    file.close();
    return data;
};

Data3D read_data_3D( std::string filepath ) {
    Data3D data;
    std::ifstream file( filepath );

    if ( !file.is_open() ) {
        std::cerr << "Error while reading data\n";
        return data;
    }

    std::string line;
    while ( std::getline( file, line ) ) {
        std::istringstream iss( line );
        std::string type;
        iss >> type;

        if ( type == "VERTEX_SE3:QUAT" ) {
            int index;
            double x, y, z, qx, qy, qz, qw;
            iss >> index >> x >> y >> z >> qx >> qy >> qz >> qw;

            Vertex_SE3 vertex{ .index = index, .x = x, .y = y, .z = z, .qx = qx, .qy = qy, .qz = qz, .qw = qw };
            data.vertices.push_back( vertex );
        }

        else if ( type == "EDGE_SE3:QUAT" ) {
            int indeces[2];
            double x, y, z, qx, qy, qz, qw;
            double info[21];

            iss >> indeces[0] >> indeces[1] >> x >> y >> z >> qx >> qy >> qz >> qw;
            for ( int i = 0; i < 21; ++i ) {
                iss >> info[i];
            }

            Edge_SE3 edge{ .x = x, .y = y, .z = z, .qx = qx, .qy = qy, .qz = qz, .qw = qw };
            edge.indeces[0] = indeces[0];
            edge.indeces[1] = indeces[1];
            for ( int i = 0; i < 21; ++i ) {
                edge.info[i] = info[i];
            }

            data.edges.push_back( edge );

        } else {
            std::cerr << "Unknown type: " << type << std::endl;
        }
    }
    file.close();
    return data;
};

// Print
// ---------------------------------------------------------
// Overloaded Print Functions
// ---------------------------------------------------------

// 2D Print Vertex
void print_vertex( const Vertex_SE2 &v ) {
    printf( "Vertex_SE2:\n  index=%d, x=%.3f, y=%.3f, theta=%.3f\n", v.index, v.x, v.y, v.theta );
}

// 3D Print Vertex
void print_vertex( const Vertex_SE3 &v ) {
    printf( "Vertex_SE3:\n  index=%d, x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f\n", v.index, v.x, v.y,
            v.z, v.qx, v.qy, v.qz, v.qw );
}

// 2D Print Edge
void print_edge( const Edge_SE2 &e ) {
    printf( "Edge_SE2:\n" );
    printf( "  indices: [%d, %d]\n", e.indeces[0], e.indeces[1] );
    printf( "  measurement: x=%.3f, y=%.3f, theta=%.3f\n", e.x, e.y, e.theta );
    printf( "  info: [" );
    for ( int i = 0; i < 6; ++i ) {
        printf( "%.3f%s", e.info[i], i < 5 ? ", " : "" );
    }
    printf( "]\n" );
}

// 3D Print Edge
void print_edge( const Edge_SE3 &e ) {
    printf( "Edge_SE3:\n" );
    printf( "  indices: [%d, %d]\n", e.indeces[0], e.indeces[1] );
    printf( "  measurement: x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f\n", e.x, e.y, e.z, e.qx, e.qy,
            e.qz, e.qw );
    printf( "  info: [" );
    for ( int i = 0; i < 21; ++i ) {
        printf( "%.3f%s", e.info[i], i < 20 ? ", " : "" );
    }
    printf( "]\n" );
}

template <typename DataType>
void print_data( const DataType& data, int max_index ) {
    printf( "Printing data\n" );

    for ( int i = 0; i < max_index && i < (int)data.vertices.size(); i++ ) {
        print_vertex( data.vertices[i] );
    }

    for ( int i = 0; i < max_index && i < (int)data.edges.size(); i++ ) {
        print_edge( data.edges[i] );
    }
}


// Check something
bool is_edge_loop_closure( int indeces[2] ) {
    int delta = std::abs( indeces[0] - indeces[1] );

    if ( delta >= 1 )
        return true;

    return false;
}

// Math
Eigen::Matrix3d get_information_matrix( const Edge_SE2& edge ) {
    Eigen::Matrix3d info_mat;
    const double* i = edge.info;

    info_mat << i[0], i[1], i[2],
                i[1], i[3], i[4],
                i[2], i[4], i[5];
                
    return info_mat;
}

Matrix6d get_information_matrix( const Edge_SE3& edge ) {
    const double* i = edge.info; 
    
    Matrix6d g2o_info;
    g2o_info << 
        i[0], i[1], i[2],  i[3],  i[4],  i[5],
        i[1], i[6], i[7],  i[8],  i[9],  i[10],
        i[2], i[7], i[11], i[12], i[13], i[14],
        i[3], i[8], i[12], i[15], i[16], i[17],
        i[4], i[9], i[13], i[16], i[18], i[19],
        i[5], i[10],i[14], i[17], i[19], i[20];
        
    // We swap the 3x3 blocks diagonally and anti-diagonally
    Matrix6d gtsam_info;
    gtsam_info << 
        g2o_info.block<3,3>(3,3), g2o_info.block<3,3>(3,0),
        g2o_info.block<3,3>(0,3), g2o_info.block<3,3>(0,0);
        
    return gtsam_info;
}

Eigen::Matrix3d get_covariance_matrix( const Edge_SE2& edge ) {
    Eigen::Matrix3d info_mat = get_information_matrix(edge);
    Eigen::Matrix3d cov_mat = info_mat.inverse();
    return cov_mat;
}

Matrix6d get_covariance_matrix( const Edge_SE3& edge ) {
    Matrix6d info_mat = get_information_matrix(edge);
    Matrix6d cov_mat = info_mat.inverse();
    return cov_mat;
}


// Export
template <typename VertexType>
void save_trajectory( const std::vector<VertexType> &vertices, const std::string &filename = "trajectory.csv" ) {
    std::ofstream file( filename );

    if ( !file.is_open() ) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    file << "id,x,y,theta\n";

    for ( const auto &v : vertices ) {
        if constexpr (std::is_same<VertexType, Vertex_SE2>::value) {
            file << v.index << "," << v.x << "," << v.y << "," << v.theta << "\n";
        } else if constexpr (std::is_same<VertexType, Vertex_SE3>::value)
        {
            file << v.index << "," << v.x << "," << v.y << "," << v.z << "," << v.qx << "," << v.qy << "," << v.qz
                 << "," << v.qw << "\n";
        }
    }

    file.close();
    std::cout << "Trajectory successfully saved to: " << filename << std::endl;
}
