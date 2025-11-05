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
    double torque_constant = 0.0601; //Kt
    double speed_constant = 159.0; //Kv -> Kb = 1/Kv *60/(2pi)
    double back_emf_constant = 1/speed_constant * 60.0 / (2.0 * M_PI); //Kb
    double speed_torque_gradient = 258.6;
    double mechanical_time_constant = 0.00223;
    double rotor_inertia = 0.00000862;
    //thermal data
        //not using this at the moment
    //mechanical data
    double max_permissible_speed_rpm = 10000.0;
    //further_specifications
    int pole_pairs = 8;
    
    //not_provided_data
    double damping_coefficient = rotor_inertia / mechanical_time_constant;
    
    //references
    double& J = rotor_inertia;
    double& R = terminal_resistance_phase2phase;
    double& L = terminal_inductance_phase2phase;
    double& Kt = torque_constant;
    double& Kb = back_emf_constant;
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

    double timeconstant_mechanical = J / b;
    double timeconstant_electrical = L / R;

    cout <<"timeconstant_mechanical: " << timeconstant_mechanical << " s" << endl;
    cout <<"timeconstant_electrical: " << timeconstant_electrical << " s" << endl;

    //eigenvalues
    Eigen::EigenSolver<Eigen::Matrix2d> eigencalculationsA(A); //vaiable_name_for_the_solver(constructor argument -> matrix you are analyzing)
    Eigen::VectorXcd eigenvaluesA = eigencalculationsA.eigenvalues(); 
    int numberOfEigenvaluesA = eigenvaluesA.size();
    cout << "Number of eigenvalues of A Matrix: " << numberOfEigenvaluesA << endl;
    for (int i = 0; i < eigenvaluesA.size(); ++i) {
        std::complex<double> lambda = eigenvaluesA(i);
        cout << "lambda " << (i+1) << " = " << lambda.real();
        if (lambda.imag() >= 0) {
            cout << " + i" << lambda.imag() << endl;
        } else {
            cout << " - i" << -lambda.imag() << endl;
        }
    }
    //Stability
    bool isStable = true;
    for (int i = 0; i < eigenvaluesA.size(); ++i) {
        if (eigenvaluesA(i).real() >= 0) {
            isStable = false;
            break;
        }
    }
    cout << "Stability check: " << endl;
    if (isStable) {
        cout << "The system is stable (all eigenvalues have negative real parts)." << endl;
    } else {
        cout << "The system is unstable (at least one eigenvalue has a non-negative real part)." << endl;
    }
    
    std::complex<double> lambda1 = eigenvaluesA(0);
    std::complex<double> lambda2 = eigenvaluesA(1);

    if (std::abs(lambda1.imag()) < 1e-10 && std::abs(lambda2.imag()) < 1e-10){
        cout << "no (very small) imaginary parts, system is overdamped" << endl;
        double timeconstant1 = -1.0 / lambda1.real();
        double timeconstant2 = -1.0 / lambda2.real();
        cout << "timeconstant1: " << timeconstant1 << " s" << endl;
        cout << "timeconstant2: " << timeconstant2 << " s" << endl;
        cout << "Settling time (4*tau): " << 4*std::max(timeconstant1, timeconstant2) << " s" << endl;
        } else {
            
        }

    return 0;
}
