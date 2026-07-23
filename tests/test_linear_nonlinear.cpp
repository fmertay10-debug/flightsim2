#include "test_util.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

#include "core/AirData.h"
#include "core/Channel.h"
#include "core/State.h"
#include "design/Linearizer.h"
#include "dynamics/SixDofEom.h"
#include "io/Json.h"
#include "vehicle/Vehicle.h"
#include "vehicle/VehicleFactory.h"

// Cross-validate the NONLINEAR sim against its own linearization: the
// Linearizer extracts the short-period plant by finite differences; here the
// same f(x,u) is time-integrated through SixDofEom and the measured ringing
// must match the linear eigenvalues. The two paths share the force stack but
// nothing else -- an EOM assembly error (dropped term, frame slip, wrong
// inertia coupling) shows up as a frequency or damping mismatch.

int main() {
    std::filesystem::create_directories("data/output");
    std::ofstream("data/output/_test_linnl_mass.csv")
        << "time_s,mass_kg,ixx,iyy,izz\n0,100,20,20,20\n10,100,20,20,20\n";

    const json::Value def = json::Value::parse(R"({
        "mass": {"model": "tabulated", "table": "_test_linnl_mass.csv"},
        "components": [{
            "type": "aircraft_aero", "sref_m2": 1.0, "cbar_m": 1.0,
            "bspan_m": 3.0, "cd0": 0.005, "cla": 5.0, "cma": -1.0,
            "cmq": -4.0, "clde": 0.1, "cmde": -1.0
        }]
    })");
    auto veh = vehicle::create(def, "data/output");
    ChannelTable table;
    veh->declareChannels(table);

    // --- Linear prediction: eigenvalues of the 2-state short-period A ---
    design::LinearizerOptions opt;
    opt.altitude = 1000.0;
    design::Linearizer lin(*veh, table, opt);
    const design::PlantPoint pp = lin.at(0.3);
    // A = [[Za, 1], [Ma, Mq]]: lambda = -sigma +/- i*wd
    const double sigmaLin = -(pp.Za + pp.Mq) / 2.0;
    const double wn2      = pp.Za * pp.Mq - pp.Ma;
    CHECK(wn2 > sigmaLin * sigmaLin);           // underdamped, as designed
    const double wdLin   = std::sqrt(wn2 - sigmaLin * sigmaLin);
    const double zetaLin = sigmaLin / std::sqrt(wn2);

    // --- Nonlinear: integrate the same components through SixDofEom.
    // Gravity-free and frozen atmosphere, matching the linearization's
    // assumptions exactly; atmosphere numbers come FROM the PlantPoint so
    // both paths see the identical flight condition. ---
    const double V0  = pp.V;
    const double rho = 2.0 * pp.qbar / (V0 * V0);
    const double a0  = pp.V / 0.3;
    const double alpha0 = 2.0 * M_PI / 180.0;

    State s;
    s.velocity = Vector3(V0 * std::cos(alpha0), 0.0, V0 * std::sin(alpha0));
    const ChannelValues u(table);           // elevator held at 0
    const SixDofEom eom;
    const MassState ms = veh->massState(0.0);
    const double dt = 2e-4, tEnd = 1.5;

    std::vector<double> tq, qq;             // q(t) history
    for (double t = 0.0; t < tEnd; t += dt) {
        const Vector3 vB = s.attitude.rotate(s.velocity);
        AirData air;
        air.atmosphere.density    = rho;
        air.atmosphere.soundSpeed = a0;
        air.velocityBody = vB;
        air.airspeed     = vB.norm();
        air.alpha        = std::atan2(vB.z, vB.x);
        air.beta         = std::atan2(vB.y, vB.x);
        air.mach         = air.airspeed / a0;
        air.qbar         = 0.5 * rho * air.airspeed * air.airspeed;

        const ComponentContext ctx{ s, air, opt.altitude, dt, ms.xcg };
        Wrench w;
        for (const auto& c : veh->components()) {
            const Wrench cw = c->computeWrench(ctx, u, nullptr);
            w.force  = w.force + cw.force;
            w.moment = w.moment + cw.moment;
        }
        s = eom.solve(s, w.force, w.moment, ms.mass, ms.inertia, dt);
        tq.push_back(s.time);
        qq.push_back(s.angularRate.y);
    }

    // --- Measure the ringing: damped frequency from zero-crossing spacing,
    // decay rate from the ratio of successive |q| peaks. ---
    std::vector<double> crossings;
    std::vector<double> peakT, peakQ;
    double best = 0.0, bestT = 0.0;
    for (std::size_t i = 1; i < qq.size(); ++i) {
        if ((qq[i - 1] < 0.0) != (qq[i] < 0.0)) {
            const double f = qq[i - 1] / (qq[i - 1] - qq[i]);
            crossings.push_back(tq[i - 1] + f * (tq[i] - tq[i - 1]));
            if (!crossings.empty() && best != 0.0) {
                peakT.push_back(bestT);
                peakQ.push_back(std::abs(best));
            }
            best = 0.0;
        }
        if (std::abs(qq[i]) > std::abs(best)) { best = qq[i]; bestT = tq[i]; }
    }
    CHECK(crossings.size() >= 4);
    CHECK(peakQ.size() >= 3);

    double sumHalf = 0.0;
    for (std::size_t i = 1; i < crossings.size(); ++i)
        sumHalf += crossings[i] - crossings[i - 1];
    const double wdMeas = M_PI / (sumHalf / (crossings.size() - 1));

    double sumSigma = 0.0;
    for (std::size_t i = 1; i < peakQ.size(); ++i)
        sumSigma += std::log(peakQ[i - 1] / peakQ[i]) /
                    (peakT[i] - peakT[i - 1]);
    const double sigmaMeas = sumSigma / (peakQ.size() - 1);
    const double zetaMeas  = sigmaMeas /
                             std::sqrt(sigmaMeas * sigmaMeas + wdMeas * wdMeas);

    std::printf("test_linear_nonlinear: wd lin %.4f meas %.4f rad/s, "
                "zeta lin %.4f meas %.4f\n", wdLin, wdMeas, zetaLin, zetaMeas);
    CHECK_NEAR(wdMeas / wdLin, 1.0, 0.03);       // frequency within 3%
    CHECK_NEAR(zetaMeas, zetaLin, 0.05);         // damping within 0.05 abs

    std::printf("test_linear_nonlinear: all checks passed\n");
    return 0;
}
