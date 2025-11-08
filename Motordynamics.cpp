#include <iostream>   // For std::cout (console output)
#include <Eigen/Dense> // Eigen library for matrix operations
#include <iomanip>    // For std::setprecision, std::fixed (formatting)
#include <cmath>      // For mathematical functions (sqrt, exp, etc.)
#include <vector>    // For std::vector
#include <yaml-cpp/yaml.h>

class MAXON_BLDC_MOTOR_SIMULATION {
    private:
        double Vnom;    // nominal voltage
        double wT0;     // no load speed rpm
        double IT0;     // no load current
        double wTn;     // nominal speed
        double Tn;      // nominal torque
        double In;      // nominal current
        double Ts;      // stall torque
        double Is;      // stall current
        double eff_max; // max efficiency
        double R;       // terminal resistance phase2phase
        double L;       // terminal inductance phase2phase
        double Kt;      // torque constant
        double Kv;      // speed constant
        double dwdT;    // speed torque gradient
        double tau_mech;// mechanical time constant
        double J;       // rotor inertia
        double wTmax;   // max permissible speed rpm
        int p;          // pole pairs
        double Kb;      // back-emf constant
        double b;       // damping coefficient

    public:
        MAXON_BLDC_MOTOR_SIMULATION(const YAML::Node &node) //reference to the yaml node motor_values
        {
            Vnom = node["nominal_voltage"].as<double>();
            wT0 = node["no_load_speed_rpm"].as<double>();
            IT0 = node["no_load_current"].as<double>();
            wTn = node["nominal_speed"].as<double>();
            Tn = node["nominal_torque"].as<double>();
            In = node["nominal_current"].as<double>();
            Ts = node["stall_torque"].as<double>();
            Is = node["stall_current"].as<double>();
            eff_max = node["max_efficiency"].as<double>();
            R = node["terminal_resistance_phase2phase"].as<double>();
            L = node["terminal_inductance_phase2phase"].as<double>();
            Kt = node["torque_constant"].as<double>();
            Kv = node["speed_constant"].as<double>();
            dwdT = node["speed_torque_gradient"].as<double>();
            tau_mech = node["mechanical_time_constant"].as<double>();
            J = node["rotor_inertia"].as<double>();
            wTmax = node["max_permissible_speed_rpm"].as<double>();
            p = node["pole_pairs"].as<int>();

            // Derived parameters
            Kb = 1.0 / Kv * 60.0 / (2.0 * M_PI); // back-emf constant
            b  = J / tau_mech;                    // damping coefficient
        };
        void printMotorInfo() const {
        std::cout << "Motor nominal voltage: " << Vnom << " V\n";
        std::cout << "No-load speed: " << wT0 << " RPM\n";
        std::cout << "No-load current: " << IT0 << " A\n";
        std::cout << "Nominal speed: " << wTn << " RPM\n";
        std::cout << "Nominal torque: " << Tn << " Nm\n";
        std::cout << "Nominal current: " << In << " A\n";
        std::cout << "Stall torque: " << Ts << " Nm\n";
        std::cout << "Stall current: " << Is << " A\n";
        std::cout << "Max efficiency: " << eff_max << "\n";
        std::cout << "Terminal resistance: " << R << " Ω\n";
        std::cout << "Terminal inductance: " << L << " H\n";
        std::cout << "Torque constant: " << Kt << " Nm/A\n";
        std::cout << "Speed constant: " << Kv << " RPM/V\n";
        std::cout << "Back-EMF constant: " << Kb << " Vs/rad\n";
        std::cout << "Speed-torque gradient: " << dwdT << "\n";
        std::cout << "Mechanical time constant: " << tau_mech << " s\n";
        std::cout << "Rotor inertia: " << J << " kg·m²\n";
        std::cout << "Damping coefficient: " << b << " Nm·s/rad\n";
        std::cout << "Max permissible speed: " << wTmax << " RPM\n";
        std::cout << "Pole pairs: " << p << "\n";
    }
};

int main() {
    YAML::Node motor_values = YAML::LoadFile("/workspaces/MotorDynamics/ECXFL42M_KL_A_HTQ_48V.yaml");

    MAXON_BLDC_MOTOR_SIMULATION motor(motor_values);

    motor.printMotorInfo();

    return 0;
}

