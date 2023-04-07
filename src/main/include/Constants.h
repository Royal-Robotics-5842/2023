#pragma once

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/area.h>
#include <units/capacitance.h>
#include <units/charge.h>
#include <units/concentration.h>
#include <units/conductance.h>
#include <units/current.h>
#include <units/curvature.h>
#include <units/data.h>
#include <units/data_transfer_rate.h>
#include <units/density.h>
#include <units/dimensionless.h>
#include <units/energy.h>
#include <units/force.h>
#include <units/frequency.h>
#include <units/illuminance.h>
#include <units/impedance.h>
#include <units/inductance.h>
#include <units/length.h>
#include <units/luminous_flux.h>
#include <units/luminous_intensity.h>
#include <units/magnetic_field_strength.h>
#include <units/magnetic_flux.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/power.h>
#include <units/pressure.h>
#include <units/radiation.h>
#include <units/solid_angle.h>
#include <units/substance.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/volume.h>

class Constants
{

public:
    static constexpr double kP = 0.00025637;
    static constexpr double kF = 0.2380952380952381;

    static constexpr double kWheelDiameter = 0.1524;

    static constexpr double kTurnP = 1/180; //max output/max error
    static constexpr double kAutoP = 3.6/15;

    static constexpr auto kArmS = 0.12855 * 1_V;
    static constexpr auto kArmG = 0.14945 * 1_V;
    static constexpr auto kArmV = 0.050359 * 1_V / 1_rad_per_s;

    static constexpr auto kArmGearRatio = 228.5;
    static constexpr auto kLowGearRatio = 10.75;

};