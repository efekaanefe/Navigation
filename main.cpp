#include "scripts/graph_slam.hpp"
#include "scripts/utils.hpp"

#include <iostream>
#include <string>
#include <vector>

void run_q1_2D() {
    // Read data file
    std::string filepath = "../data/input_INTEL_g2o.g2o";
    Data2D data;
    data = read_data_2D( filepath );
    print_data( data, 3 );

    // Max vertices: 1228 | Max edges: 1483

    // Slam algorithm
    std::vector<Vertex_SE2> optimized_traj = solve_slam( &data );
    std::vector<Vertex_SE2> optimized_traj_inc = solve_slam( &data, true );

    // Save raw and predicted trajectories as csv
    save_trajectory( data.vertices, "../results/2d_raw.csv" );
    save_trajectory( optimized_traj, "../results/2d_batch.csv" );
    save_trajectory( optimized_traj_inc, "../results/2d_incremental.csv" );
}

void run_q2_3D() {

    // Read data file
    std::string filepath = "../data/parking-garage.g2o";
    Data3D data;
    data = read_data_3D( filepath );
    print_data( data, 3 );

    // Max vertices: 1661 | Max edges: 6275

    // Slam algorithm
    std::vector<Vertex_SE3> optimized_traj = solve_slam( &data );
    std::vector<Vertex_SE3> optimized_traj_inc = solve_slam( &data, true );

    // Save raw and predicted trajectories as csv
    save_trajectory( data.vertices, "../results/3d_raw.csv" );
    save_trajectory( optimized_traj, "../results/3d_batch.csv" );
    save_trajectory( optimized_traj_inc, "../results/3d_incremental.csv" );
}

int main() {

    run_q1_2D();
    run_q2_3D();

    return 0;
}
