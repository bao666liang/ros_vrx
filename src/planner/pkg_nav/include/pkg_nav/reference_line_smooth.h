#pragma once

// #include "Frenet/path_struct.h"
#include "smoothosqpproblem.h"

class PathPoint {
public:
    PathPoint() = default;
    PathPoint(double x, double y , double z,
              double theta, double kappa,
              double dkappa, double ddkappa):
                rx(x), ry(y),rz(z),
                rtheta(theta),rkappa(kappa),
                rdkappa(dkappa), rddkappa(ddkappa){}
    double x()const { return rx; }
    double y()const { return ry; }
    double z()const { return rz; }
    double theta()const { return rtheta;}
    double s()const { return rs; }
    double kappa()const { return rkappa; }
    double dkappa()const { return rdkappa; }
    double ddkappa()const { return rddkappa; }
    int laneId()const { return r_lane_id; }
    void set_x(const double x) { rx = x; }
    void set_y(const double y) { ry = y; }
    void set_z(const double z) { rz = z; }
    void set_theta(const double theta) { rtheta = theta;}
    void set_s(const double s1) { rs = s1; }
    void set_kappa(const double kappa) { rkappa = kappa; }
    void set_dkappa(const double dkappa) { rdkappa = dkappa; }
    void set_ddkappa(const double ddkappa) { rddkappa = ddkappa;}
    void set_laneId(const int laneId) { r_lane_id = laneId; }
    void CopyFrom(const PathPoint &point) {
        rx = point.x();
        ry = point.y();
        rz = point.z();
        rtheta = point.theta();
        rkappa = point.kappa();
        rs = point.s();
        rdkappa = point.dkappa();
        rddkappa = point.ddkappa();
        r_lane_id = point.laneId();
    }
private:
  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
  // direction on the x-y plane
  double rtheta = 0.0;
  // curvature on the x-y planning
  double rkappa = 0.0;
  // accumulated distance from beginning of the path
  double rs = 0.0;
  // derivative of kappa w.r.t s.
  double rdkappa = 0.0;
  // derivative of derivative of kappa w.r.t s.
  double rddkappa = 0.0;
  // The lane ID where the path point is on
  int r_lane_id = 0.0;
};

class ReferenceLineSmooth{
public:
    ReferenceLineSmooth() = default;
    bool Smooth(std::vector<PathPoint> &raw_path, std::vector<PathPoint> *smooth_path);
};
