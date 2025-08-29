#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>

// Constants
const int N = 100;              // Number of agents
const double L = 10.0;          // Size of the square domain (periodic boundary)
const double R = 1.0;           // Interaction radius
const double v0 = 0.03;         // Speed of each agent
const double eta = 0.1;         // Noise strength (0 = no noise, 1 = max noise)
const double dt = 1.0;          // Time step
const int steps = 100;         // Total number of time steps

// Random number generator
std::mt19937 rng(42);  // Fixed seed for reproducibility
std::uniform_real_distribution<double> uniform_angle(0.0, 2.0 * M_PI);
std::uniform_real_distribution<double> uniform_noise(-eta / 2.0, eta / 2.0);

// Agent structure
struct Agent {
    double x, y;    // Position
    double theta;   // Direction angle (in radians)

    // Move agent in its current direction
    void move() {
        x += v0 * cos(theta);
        y += v0 * sin(theta);

        // Apply periodic boundary conditions
        if (x < 0) x += L;
        if (x >= L) x -= L;
        if (y < 0) y += L;
        if (y >= L) y -= L;
    }
};

// Compute distance with periodic boundaries
double distance(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;

    // Minimum image convention
    if (dx > L/2) dx -= L;
    if (dx < -L/2) dx += L;
    if (dy > L/2) dy -= L;
    if (dy < -L/2) dy += L;

    return sqrt(dx*dx + dy*dy);
}

int main() {
    // Initialize agents
    std::vector<Agent> agents(N);
    std::uniform_real_distribution<double> uniform_pos(0.0, L);
    for (int i = 0; i < N; ++i) {
        agents[i].x = uniform_pos(rng);
        agents[i].y = uniform_pos(rng);
        agents[i].theta = uniform_angle(rng);
    }

    // Output file
    std::ofstream output("vicsek_output.csv");

    // Simulation loop
    for (int step = 0; step < steps; ++step) {
        // Write current positions to file
        for (const auto& a : agents) {
            output << step << "," << a.x << "," << a.y << "\n";
        }

        // Compute new directions
        std::vector<double> new_thetas(N);
        for (int i = 0; i < N; ++i) {
            double sum_sin = 0.0;
            double sum_cos = 0.0;

            for (int j = 0; j < N; ++j) {
                if (distance(agents[i].x, agents[i].y, agents[j].x, agents[j].y) < R) {
                    sum_cos += cos(agents[j].theta);
                    sum_sin += sin(agents[j].theta);
                }
            }

            double avg_theta = atan2(sum_sin, sum_cos);
            new_thetas[i] = avg_theta + uniform_noise(rng);  // Add noise
        }

        // Update angles and move
        for (int i = 0; i < N; ++i) {
            agents[i].theta = new_thetas[i];
            agents[i].move();
        }
    }

    output.close();
    std::cout << "Simulation complete. Output saved to vicsek_output.csv.\n";

    return 0;
}
