// Read-only: flange pose from the monitoring stream (no control request, works while the TP has control).
// Prints one JSON line: {"flange_mm":[x,y,z],"tool_z":[r02,r12,r22],"posx":[x,y,z,A,B,C]}
#define DRCF_VERSION 2
#include "DRFLEx.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
using namespace DRAFramework;
static std::mutex m;
static float pos[6], R[3][3];
static std::atomic<bool> got{false};
int main(int argc, char** argv) {
    std::string ip = argc > 1 ? argv[1] : "192.168.137.100";
    CDRFLEx r;
    r.set_on_monitoring_data_ex([](const LPMONITORING_DATA_EX d) {
        std::lock_guard<std::mutex> lk(m);
        for (int i = 0; i < 6; ++i) pos[i] = d->_tCtrl._tTask._fActualPos[1][i];  // 1 = flange
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) R[a][b] = d->_tCtrl._tTask._fRotationMatrix[a][b];
        got = true;
    });
    if (!r.open_connection(ip)) { std::fprintf(stderr, "connect failed\n"); return 1; }
    r.setup_monitoring_version(1);
    for (int i = 0; i < 50 && !got; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    if (!got) { std::fprintf(stderr, "no monitoring data\n"); return 1; }
    std::lock_guard<std::mutex> lk(m);
    std::printf("{\"flange_mm\":[%.3f,%.3f,%.3f],\"tool_z\":[%.5f,%.5f,%.5f],\"posx\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]}\n",
                pos[0], pos[1], pos[2], R[0][2], R[1][2], R[2][2], pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    r.close_connection();
    return 0;
}
