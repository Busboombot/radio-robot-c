#include "Odometry.h"
#include <math.h>

Odometry::Odometry()
    : _x(0.0f), _y(0.0f), _headingRad(0.0f)
{
}

void Odometry::update(float dL_mm, float dR_mm, float trackwidthMm)
{
    float dCenter = (dL_mm + dR_mm) * 0.5f;
    float dTheta  = (dR_mm - dL_mm) / trackwidthMm;

    _x          += dCenter * cosf(_headingRad);
    _y          += dCenter * sinf(_headingRad);
    _headingRad += dTheta;
}

void Odometry::getPose(int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg) const
{
    x_mm = static_cast<int32_t>(_x);
    y_mm = static_cast<int32_t>(_y);

    float cdeg = _headingRad * RAD_TO_CDEG;
    if (cdeg >  18000.0f) cdeg =  18000.0f;
    if (cdeg < -18000.0f) cdeg = -18000.0f;
    h_cdeg = static_cast<int32_t>(cdeg);
}

void Odometry::setPose(int32_t x_mm, int32_t y_mm, int32_t h_cdeg)
{
    _x          = static_cast<float>(x_mm);
    _y          = static_cast<float>(y_mm);
    _headingRad = static_cast<float>(h_cdeg) * CDEG_TO_RAD;
}

void Odometry::zero()
{
    setPose(0, 0, 0);
}
