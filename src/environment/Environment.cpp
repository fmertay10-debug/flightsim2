#include "environment/Environment.h"

#include <cmath>
#include <utility>

namespace {
    constexpr double G0     = 9.80665;      // standard gravity [m/s^2]
    constexpr double R_AIR  = 287.05287;    // gas constant, dry air [J/(kg K)]
    constexpr double GAMMA  = 1.4;          // ratio of specific heats
    constexpr double R_GEOP = 6356766.0;    // Earth radius for geopotential conversion [m]

    // One ISA layer: base geopotential altitude, lapse rate, base T, base P.
    struct IsaLayer {
        double Hb;   // [m]
        double Lb;   // [K/m]
        double Tb;   // [K]
        double Pb;   // [Pa]
    };

    // Standard atmosphere, sea level to 71 km (model ceiling 84.852 km).
    constexpr IsaLayer LAYERS[] = {
        {     0.0, -0.0065, 288.15, 101325.0     },
        { 11000.0,  0.0,    216.65,  22632.06    },
        { 20000.0,  0.001,  216.65,   5474.889   },
        { 32000.0,  0.0028, 228.65,    868.0187  },
        { 47000.0,  0.0,    270.65,    110.9063  },
        { 51000.0, -0.0028, 270.65,     66.93887 },
        { 71000.0, -0.002,  214.65,      3.956420}
    };
    constexpr int N_LAYERS = 7;
}

Environment::Environment(std::unique_ptr<GravityModel> gravity,
                         std::unique_ptr<WindModel>    wind)
    : gravity_(std::move(gravity)), wind_(std::move(wind)) {}

AtmosphereState Environment::atmosphere(double altitude) const {
    // Geometric -> geopotential altitude.
    const double H = R_GEOP * altitude / (R_GEOP + altitude);

    // Highest layer whose base is at or below H.
    int idx = 0;
    for (int k = 0; k < N_LAYERS; ++k) {
        if (H >= LAYERS[k].Hb) idx = k;
        else break;
    }
    const IsaLayer& L = LAYERS[idx];

    const double T = L.Tb + L.Lb * (H - L.Hb);

    double P;
    if (L.Lb == 0.0) {                                   // isothermal layer
        P = L.Pb * std::exp(-G0 * (H - L.Hb) / (R_AIR * L.Tb));
    } else {                                             // gradient layer
        P = L.Pb * std::pow(T / L.Tb, -G0 / (R_AIR * L.Lb));
    }

    const double rho = P / (R_AIR * T);
    const double a   = std::sqrt(GAMMA * R_AIR * T);

    return AtmosphereState{ rho, P, T, a };
}

double Environment::gravity(double altitude) const {
    return gravity_->gravity(altitude);
}

Vector3 Environment::wind(const Vector3& positionNed, double time) const {
    return wind_->wind(positionNed, time);
}
