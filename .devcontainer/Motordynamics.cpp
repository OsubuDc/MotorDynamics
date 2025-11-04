#include <iostream>   // For std::cout (console output)
#include <Eigen/Dense> // Eigen library for matrix operations
#include <iomanip>    // For std::setprecision, std::fixed (formatting)
#include <cmath>      // For mathematical functions (sqrt, exp, etc.)

int main() {
    //ECXFL42M KL A HTQ 48V
    //values at nominal voltage
    double nominal_voltage = 48.0; 
    double no_load_speed_rpm = 7560.0;
    double no_load_current = 0.163;
    double nominal_speed = 6070.0;
    double nominal_torque = 0.221;
    double nominal_current = 3.53;
    double stall_torque = 1.140;
    double stall_current = 51.4;
    double max_efficiency = 0.84098;
    //characteristics
    double terminal_resistance_phase2phase = 0.933;
    double terminal_inductance_phase2phase = 0.000698;
    double torque_constant = 0.0601;
    double speed_constant = 159.0;
    double mechanical_time_constant = 0.00223;
    double rotor_inertia = 0.00000862;
    //thermal data
        //not using this at the moment
    //mechanical data
    double max_permissible_speed_rpm = 10000.0;
    //further_specifications
    int pole_pairs = 8;
    
    //not_provided_data
    double damping_coefficient = 0.001;
    
    //references
    double& J = rotor_inertia;
    double& R = terminal_resistance_phase2phase;
    double& L = terminal_inductance_phase2phase;
    double& Kt = torque_constant;
    double& Kb = torque_constant; // assuming Kb = Kt
    double& b = damping_coefficient;

    //StateSpace
    Eigen::Matrix2d A;
    A << -R/L, -Kb/L, Kt/J, -b/J;

    using namespace std;
    cout << "Motor Parameters" << endl;
    cout << "Rotor inertia J: " << J << " kgm²" << endl;
    cout << "Resistance R: " << R << " Ohm" << endl;
    cout << "Inductance L: " << L << " H" << endl;
    cout << "Torque constant Kt: " << Kt << " Nm/A" << endl;
    cout << "Back EMF constant Kb: " << Kb << " Vs/rad" << endl;
    cout << "Damping coefficient b: " << b << " Nm/(rad/s)" << endl;
    cout << "State-Space A Matrix: " << endl << A << endl;
}