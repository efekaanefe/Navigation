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

    // Save GT and predicted trajectories as csv
    save_trajectory( data.vertices, "../results/initial_traj.csv" );
    save_trajectory( optimized_traj, "../results/optimized_traj.csv" );
    save_trajectory( optimized_traj_inc, "../results/optimized_traj_inc.csv" );
}

void run_q2_3D() {

    // Read data file
    std::string filepath = "../data/parking-garage.g2o";
    Data3D data;
    data = read_data_3D( filepath );
    print_data( data, 3 );

    // Slam algorithm
    std::vector<Vertex_SE3> optimized_traj = solve_slam( &data );
    std::vector<Vertex_SE3> optimized_traj_inc = solve_slam( &data, true );

    // Save GT and predicted trajectories as csv
    save_trajectory( data.vertices, "../results/initial_traj.csv" );
    save_trajectory( optimized_traj, "../results/optimized_traj.csv" );
    save_trajectory( optimized_traj_inc, "../results/optimized_traj_inc.csv" );
}

int main() {

    run_q1_2D();
    run_q2_3D();

    return 0;
}
