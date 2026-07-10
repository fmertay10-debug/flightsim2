#include "sim/CsvLogger.h"

#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <stdexcept>

CsvLogger::CsvLogger(int entityId, const std::string& path, int decimation)
    : entityId_(entityId), decimation_(decimation < 1 ? 1 : decimation)
{
    const std::filesystem::path p(path);
    if (p.has_parent_path())
        std::filesystem::create_directories(p.parent_path());

    out_.open(path, std::ios::trunc);
    if (!out_)
        throw std::runtime_error("CsvLogger: cannot open '" + path + "'");

    out_ << "time,x,y,z,vx,vy,vz,phi,theta,psi,p,q,r,"
            "elevator,aileron,rudder,throttle,"
            "elevator_cmd,aileron_cmd,rudder_cmd,throttle_cmd,"
            "tvc_pitch,tvc_yaw,"
            "alpha,beta,mach,airspeed,altitude,mass,thrust,"
            "pitch_sp,roll_sp,heading_sp,altitude_sp,speed_sp\n";
    out_ << std::setprecision(8);
}

namespace {
// Setpoints are optional per channel; write NaN when a channel isn't commanded
// so analysis tools can tell "not tracked" from "tracked to zero".
double sp(const std::optional<double>& v) {
    return v ? *v : std::numeric_limits<double>::quiet_NaN();
}
}

void CsvLogger::onStep(const Telemetry& t) {
    if (t.id != entityId_) return;
    if (count_++ % decimation_ != 0) return;

    const State& s = t.state;
    const Vector3 e = s.eulerAngles();
    out_ << s.time << ','
         << s.position.x << ',' << s.position.y << ',' << s.position.z << ','
         << s.velocity.x << ',' << s.velocity.y << ',' << s.velocity.z << ','
         << e.x << ',' << e.y << ',' << e.z << ','
         << s.angularRate.x << ',' << s.angularRate.y << ',' << s.angularRate.z << ','
         << t.control.elevator << ',' << t.control.aileron << ','
         << t.control.rudder << ',' << t.control.throttle << ','
         << t.controlCmd.elevator << ',' << t.controlCmd.aileron << ','
         << t.controlCmd.rudder << ',' << t.controlCmd.throttle << ','
         << t.control.tvcPitch << ',' << t.control.tvcYaw << ','
         << t.air.alpha << ',' << t.air.beta << ',' << t.air.mach << ','
         << t.air.airspeed << ',' << s.altitude() << ','
         << t.mass << ',' << t.thrust << ','
         << sp(t.setpoint.pitch) << ',' << sp(t.setpoint.roll) << ','
         << sp(t.setpoint.heading) << ',' << sp(t.setpoint.altitude) << ','
         << sp(t.setpoint.speed) << '\n';
}

void CsvLogger::onFinish() {
    out_.flush();
}
