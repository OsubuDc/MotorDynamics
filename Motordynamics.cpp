#include <iostream>   // For std::cout (console output)
#include <Eigen/Dense> // Eigen library for matrix operations
#include <iomanip>    // For std::setprecision, std::fixed (formatting)
#include <cmath>      // For mathematical functions (sqrt, exp, etc.)
#include <vector>    // For std::vector
#include <yaml-cpp/yaml.h>
#include <fstream>

struct SimulationDataPoint {    //struct is used to group several variables together under a single name. Public by default
    double time;
    double current;
    double speed_rad;
    double speed_rpm;
    double torque;
    double voltage;
    double load_torque;
};

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
        double TmaxNomv;    //max torque at n = 0

        Eigen::Vector2d x;
        Eigen::Matrix2d A;
        Eigen::Matrix2d B;
        Eigen::Matrix2d C;
        Eigen::Matrix2d D;

        double current_time;
        double dt;

        bool current_limit_reached = false;

        bool recording_enabled = false;
        std::vector<SimulationDataPoint> recorded_data_vector;

    public:
        MAXON_BLDC_MOTOR_SIMULATION(const YAML::Node &node, double timestep = 0.0001) //reference to the yaml node motor_values
                                    :dt(timestep), current_time(0.0) //timestep = default value when you don't assign when Motor m1(0.05);
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
            b  = Kt*IT0/(wT0* 2.0*M_PI/60.0);  // damping coefficient
            TmaxNomv = -(0-Kv*48.0)/(dwdT);

            buildStateSpaceModel();

            //ini motor at rest
            x = Eigen::Vector2d::Zero();

            std::cout << "Class object initialized with dt = " << dt << "\n\n";

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
        std::cout << "TmaxNom: " << TmaxNomv << "\n\n";
    }

    void buildStateSpaceModel() {
        A << -R / L,    -Kb / L,
                Kt / J,   -b / J;
        B << 1.0/L, 0.0,
                0.0, -1.0/J;
        C << 1.0, 0.0,
                0.0, 1.0;
        D = Eigen::Matrix2d::Zero();

        std::cout << "State-Space Model Matrices:\n"; // \n for new line
        std::cout << "A Matrix:\n" << A << "\n\n";
        std::cout << "B Matrix:\n" << B << "\n\n";
        std::cout << "C Matrix:\n" << C << "\n\n";
        std::cout << "D Matrix:\n" << D << "\n\n";
    }

    void stepRK4(double voltage, double load_torque){
        Eigen::Vector2d u;
        u << voltage, load_torque;

        Eigen::Vector2d k1 = A*x + B*u;
        Eigen::Vector2d k2 = A*(x+0.5*dt*k1)+B*u;
        Eigen::Vector2d k3 = A*(x+0.5*dt*k2)+B*u;
        Eigen::Vector2d k4 = A*(x+dt*k3)+B*u;

        x += (k1 + 2*k2 + 2*k3 + k4) * (dt/6.0);

        if (x(0) > Is) {
            x(0) = Is;
            if (!current_limit_reached) {
                std::cout << "Current limit of " << Is << " A reached" << std::endl;
                current_limit_reached = true; // set flag so it only prints once
            }
        }

        current_time += dt;

        if (recording_enabled) { //recording the state for each timestep
            SimulationDataPoint point;
            point.time = current_time;
            point.current = x(0);
            point.speed_rad = x(1);
            point.speed_rpm = x(1)*60.0/(2.0*M_PI);
            point.torque = Kt*x(0) - b*x(1);
            point.voltage = voltage;
            point.load_torque = load_torque;
            recorded_data_vector.push_back(point);
        }
    }

    void StartRecording() {
        recording_enabled = true;
        recorded_data_vector.clear();
    }

    void StopRecording() {
        recording_enabled = false;
    }

    void exportToCSV(const std::string& filename) const { // const so dont change and then pass by reference. Filename is an existing string owned by whoever called the function (see later)
        //ofstream is part of fstream and is used to write data to files. Behaves like cout but instead of printing to the terminal it writes to a file.
        std::ofstream file(filename); //open file with the name "filename" in working directory. If file doesn't exist it makes one, if it does exist it overwrites the existing file.
        if (!file.is_open()) {
            std::cerr << "Error: Could not open the file " << filename << std::endl; //cerr is like cout but for errors
            return;
        }
        file << "time,current,speed_rad,speed_rpm,torque,voltage,load_torque\n"; //header in this context are column labels, every row that follows contains the data. Its not the same kind of header as #include <> 
        for (const auto& point : recorded_data_vector) { //auto makes sure to get the right variable type and &point is a reference to the vector points.
            file << std::fixed << std::setprecision(6) //setprecision on 'file' so dont need to reset later.
            << point.time << ","
            << point.current << ","
            << point.speed_rad << ","
            << point.speed_rpm << ","
            << point.torque << ","
            << point.voltage << ","
            << point.load_torque << "\n";
        }
    
        file.close();
        std::cout << "Data exported to " << filename << " (" << recorded_data_vector.size() << " points)\n";
    }


    //(*empty*) means that it takes no arguments. x is in private so we need getter functions to grab the values. We can also do calculations on the fly in these functions.
   //we made get functions in the public part of the class which expose private variables to outside the class
    //now we can do motor.getCurrentTime for example.
    double getCurrent() const { 
        return x(0);
    }

    double getSpeedRAD() const {
        return x(1);
    }

    double getSpeedRPM() const {
        return x(1)*60.0/(2.0*M_PI);
    }

    double getCurrentTime() const {
        return current_time;
    }

    double getUsefulMotorTorque () {
        return Kt * x(0) - b*x(1);
    }

    void resetNoIC() {
        x = Eigen::Vector2d::Zero();
        current_time = 0.0;
    }

    void resetIC(double init_current, double init_vel_rad) {
        x << init_current, init_vel_rad;
        current_time = 0.0;
    }

    double getKv() const { 
        return Kv;
    }

    double getdwdT() const {
        return dwdT;
    }

    Eigen::Vector2d getState() const {
        return x;
    }

    void printState() {
        std::cout   << std::fixed << std::setprecision(4)
                    << "t= " << current_time << " s" << std::endl
                    << "i= " << x(0) << " A" << std::endl
                    << "w= " << x(1) << " rad/s" << std::endl
                    << "n= " << x(1)*60.0/(2.0*M_PI) << " rpm" << std::endl
                    << "T= " << Kt * x(0) - b*x(1) << "Nm" << std::endl
                    << std::endl;
        std::cout << std::defaultfloat;
    }

    void setNewTimeStep(double new_dt) {
        dt = new_dt;
    }
};

void torque_step(MAXON_BLDC_MOTOR_SIMULATION& motor, double voltage, double duration, double load_torque) {
    std::cout << "\nStep response " << voltage << "V\n";
    motor.resetNoIC();
    motor.StartRecording();
    
    int print_interval = 10000;
    int step_count = 0;

    while (motor.getCurrentTime() < duration) {
        motor.stepRK4(voltage, load_torque);
        if (step_count % print_interval ==0) {
            motor.printState();
        }
        step_count++;
    }
    motor.StopRecording();

    std::string filename = "step_response_"+std::to_string(int(voltage))+"V.csv";
    motor.exportToCSV(filename);
}


void torque_sweep(MAXON_BLDC_MOTOR_SIMULATION& motor, double voltage, double Torque_Min, double Torque_Max, int num_data_points) {
    std::cout << "\nTorque sweep: " << voltage << "V\n";
    std::cout << "Torque range: " << Torque_Min << " to " << Torque_Max << " Nm\n";
    std::vector<SimulationDataPoint> data_sweep_vector;
    for (int i = 0; i < num_data_points; i++) {
        double load_torque = Torque_Min + (Torque_Max-Torque_Min)*i / num_data_points;
        motor.resetNoIC();
        double settle_time = 0.1;
        while (motor.getCurrentTime() < settle_time) {
            motor.stepRK4(voltage, load_torque);
        }

        SimulationDataPoint point;
        point.time = motor.getCurrentTime();
        point.current = motor.getCurrent();
        point.speed_rad = motor.getSpeedRAD();
        point.speed_rpm = motor.getSpeedRPM();
        point.torque = motor.getUsefulMotorTorque();
        point.voltage = voltage;
        point.load_torque = load_torque;
        data_sweep_vector.push_back(point);

        if (i % 10 == 0) {
            std::cout << "Torque: " << load_torque << "Nm\n" << point.speed_rpm << "rpm\n\n";
        }
    }
    std::string filename = "torque_sweep_" + std::to_string((int)voltage) +"V.csv"; //(int) is a cast, it converst the double to an int
    std::ofstream file(filename);
    file << "current,speed_rad,speed_rpm,torque,voltage,load_torque\n";
    for (const auto& point : data_sweep_vector) {
        file << point.current << "," //we always reset state vector and wait for set time to get to steady state so time is useless.
            << point.speed_rad << ","
            << point.speed_rpm << ","
            << point.torque << ","
            << point.voltage << ","
            << point.load_torque <<"\n";
    }
    file.close();
    std::cout << "sweep data vector exported to " << filename << "\n";
}



int main() {
    YAML::Node motor_values = YAML::LoadFile("/workspaces/MotorDynamics/ECXFL42M_KL_A_HTQ_48V.yaml");

    //double dt = 0.05;
    
    MAXON_BLDC_MOTOR_SIMULATION motor(motor_values); //(motor_values, dt)
    
    motor.printMotorInfo();
    
    double Tmax_48V = motor.getKv() * 48.0 / motor.getdwdT();
    torque_sweep(motor, 48.0, 0.0, Tmax_48V, 50);
    double load_torque = 0.0;
    double voltage = 48.0;
    double duration = 10;
    torque_step(motor, voltage, duration, load_torque);

// motor voltage and end time simulation
    //std::cout << "Motor simulation: 48V - NO LOAD" << std::endl;
    // motor.resetNoIC();
    // double voltage = 24.0;
    // double load_torque = 3.1;
    // double simulation_duration = 10; //s
    // int print_interval = 1000; //every 100 steps
    // int step_count = 0;

    // double Kv = motor.getKv();
    // double dwdT = motor.GetdwdT();

    // double Tmax = -(0-Kv*voltage)/(dwdT);

    // std::cout << "Tmax for " << voltage << "V is: " << Tmax << "\n\n";

    // while (motor.getCurrentTime()<simulation_duration) {
    //     motor.stepRK4(voltage,load_torque);
    //     if (step_count % print_interval == 0) {
    //         motor.printState();
    //     }
    //     step_count++;
    // }

//torque sweeps for plotting
    
    return 0;
}


