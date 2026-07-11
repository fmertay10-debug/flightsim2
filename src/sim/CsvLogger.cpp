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
    out_ << std::setprecision(8);
}

void CsvLogger::writeHeader(const Telemetry* t) {
    headerWritten_ = true;
    const ChannelTable* table = t ? t->channels : nullptr;
    if (table) {
        elevator_ = table->find(channels::kElevator);
        aileron_  = table->find(channels::kAileron);
        rudder_   = table->find(channels::kRudder);
        throttle_ = table->find(channels::kThrottle);
        tvcPitch_ = table->find(channels::kTvcPitch);
        tvcYaw_   = table->find(channels::kTvcYaw);
        for (int i = 0; i < table->size(); ++i) {
            const std::string& n = table->def(i).name;
            if (n != channels::kElevator && n != channels::kAileron &&
                n != channels::kRudder && n != channels::kThrottle &&
                n != channels::kTvcPitch && n != channels::kTvcYaw)
                extras_.push_back(i);
        }
    }

    out_ << "time,x,y,z,vx,vy,vz,phi,theta,psi,p,q,r,"
            "elevator,aileron,rudder,throttle,"
            "elevator_cmd,aileron_cmd,rudder_cmd,throttle_cmd,"
            "tvc_pitch,tvc_yaw,";
    for (const int i : extras_) {
        const std::string& n = table->def(i).name;
        out_ << n << ',' << n << "_cmd,";
    }
    out_ << "alpha,beta,mach,airspeed,altitude,mass,thrust,"
            "pitch_sp,roll_sp,heading_sp,altitude_sp,speed_sp,qbar";
    if (t && t->componentNames) {
        nComponents_ = static_cast<int>(t->componentNames->size());
        for (const std::string& n : *t->componentNames)
            out_ << ',' << n << "_fx," << n << "_fy," << n << "_fz,"
                 << n << "_mx," << n << "_my," << n << "_mz";
    }
    out_ << '\n';
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
    if (!headerWritten_) writeHeader(&t);

    const State& s = t.state;
    const Vector3 e = s.eulerAngles();
    out_ << s.time << ','
         << s.position.x << ',' << s.position.y << ',' << s.position.z << ','
         << s.velocity.x << ',' << s.velocity.y << ',' << s.velocity.z << ','
         << e.x << ',' << e.y << ',' << e.z << ','
         << s.angularRate.x << ',' << s.angularRate.y << ',' << s.angularRate.z << ','
         << t.control.get(elevator_) << ',' << t.control.get(aileron_) << ','
         << t.control.get(rudder_) << ',' << t.control.get(throttle_) << ','
         << t.controlCmd.get(elevator_) << ',' << t.controlCmd.get(aileron_) << ','
         << t.controlCmd.get(rudder_) << ',' << t.controlCmd.get(throttle_) << ','
         << t.control.get(tvcPitch_) << ',' << t.control.get(tvcYaw_) << ',';
    for (const int i : extras_)
        out_ << t.control.at(i) << ',' << t.controlCmd.at(i) << ',';
    out_ << t.air.alpha << ',' << t.air.beta << ',' << t.air.mach << ','
         << t.air.airspeed << ',' << s.altitude() << ','
         << t.mass << ',' << t.thrust << ','
         << sp(t.setpoint.pitch) << ',' << sp(t.setpoint.roll) << ','
         << sp(t.setpoint.heading) << ',' << sp(t.setpoint.altitude) << ','
         << sp(t.setpoint.speed) << ',' << t.air.qbar;
    for (int i = 0; i < nComponents_; ++i) {
        const Wrench& w = t.componentLoads[static_cast<std::size_t>(i)];
        out_ << ',' << w.force.x << ',' << w.force.y << ',' << w.force.z
             << ',' << w.moment.x << ',' << w.moment.y << ',' << w.moment.z;
    }
    out_ << '\n';
}

void CsvLogger::onFinish() {
    if (!headerWritten_) writeHeader(nullptr);
    out_.flush();
}
