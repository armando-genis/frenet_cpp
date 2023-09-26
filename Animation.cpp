#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "matplotlibcpp.h"
#include "CubicSpline1D.h"

using namespace std;
namespace plt = matplotlibcpp;

void plot_waypoints( const std::vector<Eigen::VectorXd> waypoints){
    std::vector<double> x_vals, y_vals;
    for (const auto& waypoint : waypoints) {
        x_vals.push_back(waypoint[0]);
        y_vals.push_back(waypoint[1]);
    }

    plt::plot(x_vals, y_vals, "bo-");  // Plot blue circles connected by lines
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Waypoints");
    plt::grid(true);
    plt::show();
}



int main() {
    // Given waypoints
    std::vector<double> x = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0,6.0,6.0,6.0,6.0,6.0,6.0, 5.0,4.0,3.0,2.0,1.0,0.0,-1.0,-2.0,-3.0,-4.0,-5.0,-6.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0, -6.0,-5.0,-4.0,-3.0,-2.0};
    std::vector<double> y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,2.0,3.0,4.0,5.0,6.0,7.0, 7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0, 6.0,5.0,4.0,3.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0};
    vector<Eigen::VectorXd> waypoints;
    
    // Interpolate waypoints using CubicSpline1D
    // Generating a list of indices for the waypoints:
    std::vector<double> t_values(x.size());
    std::iota(t_values.begin(), t_values.end(), 0);  // fills t_values with increasing values starting from 0

    // Create the splines using these indices as the independent variable
    CubicSpline1D spline_x(t_values, x);
    CubicSpline1D spline_y(t_values, y);

    // Now, when you interpolate, use the range of t_values:
    // std::vector<double> x_new, y_new;
    for (double t = 0; t < t_values.back(); t += 0.5) {
        Eigen::VectorXd waypoint(2);
        waypoint << spline_x.calc_der0(t), spline_y.calc_der0(t);
        waypoints.push_back(waypoint);
        // x_new.push_back(spline_x.calc_der0(t));
        // y_new.push_back(spline_y.calc_der0(t));
    }

    std::cout << "waypoints size: " << waypoints.size() << std::endl;

    // Plot interpolated waypoints
    plot_waypoints(waypoints);

    // #print the size of waypoints
    



    return 0;
}
