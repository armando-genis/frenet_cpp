#include <iostream>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <vector>
#include <numeric>  // for std::iota

#include "matplotlibcpp.h"
#include "CubicSpline1D.h"
#include "BicycleModel.h"

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


void animation_car(const vector<Eigen::VectorXd> waypoints){
    plt::ion();  
    BicycleModel vehicle(0.0, 0.0, 0.0, 1.0);


    std::vector<double> vehicle_x, vehicle_y, vehicle_theta;
    double dt = 0.1;
    for (int i = 0; i < 500; i++) {



        cout << "vehicle x: " << vehicle.getX() << endl;
        cout << "vehicle y: " << vehicle.getY() << endl;
        cout << "Vehicle yaw: " << vehicle.getYaw() << std::endl;

    
        double velocity = vehicle.getV();


        // Store vehicle data for plotting
        vehicle_x.push_back(vehicle.getX());
        vehicle_y.push_back(vehicle.getY());
        vehicle_theta.push_back(vehicle.getTheta());

        // Update vehicle state using bicycle model
        
        vehicle.update(0.3, 0.01, 0.1,0.523599); // delta (steering wheel), a (acceletarion), dt, max_steer = 30 deg

        plt::clf();
        vector<double> wp_x, wp_y;
        for (const auto& waypoint : waypoints) {
            wp_x.push_back(waypoint[0]);
            wp_y.push_back(waypoint[1]);
        }

        plt::plot(wp_x, wp_y, "ro-");
        plt::plot(vehicle_x, vehicle_y, "go");
        
        plt::named_plot("waypoints", wp_x, wp_y);
        plt::named_plot("vehicle", vehicle_x, vehicle_y);
        plt::legend();  // Show the legend to access the plot's properties
        
        plt::xlabel("X");
        plt::ylabel("Y");
        plt::title("Frenet Frame");
        plt::grid(true);

        plt::show();
        try {
            plt::pause(0.1);
        } catch (const std::runtime_error& e) {
            std::cerr << "Runtime error: " << e.what() << std::endl;
        }

    }


    // Keep the plot window open
    plt::show(true);
    
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
    }

    std::cout << "waypoints size: " << waypoints.size() << std::endl;

    // Plot interpolated waypoints
    animation_car(waypoints);

    // #print the size of waypoints
    



    return 0;
}
