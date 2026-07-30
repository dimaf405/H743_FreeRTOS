#include "test_framework.hpp"

#include "Dima/lib/motor/speed_to_pwm.hpp"

#include <limits>
#include <type_traits>

namespace {

using dima::lib::motor::ConvertStatus;
using dima::lib::motor::MotorPwmFrame;
using dima::lib::motor::MotorSpeedFrame;
using dima::lib::motor::SpeedPwmCalibration;
using dima::lib::motor::kDefaultSpeedPwmCalibration;
using dima::lib::motor::kMotorCount;
using dima::lib::motor::speed_frame_to_pwm;
using dima::lib::motor::speed_to_pwm;

static_assert(std::is_trivially_copyable<MotorSpeedFrame>::value,
              "motor speed frames must remain trivially copyable");
static_assert(std::is_trivially_copyable<MotorPwmFrame>::value,
              "motor PWM frames must remain trivially copyable");

void check_conversion(float speed,
                      uint16_t expected_pwm,
                      ConvertStatus expected_status,
                      const SpeedPwmCalibration &calibration =
                          kDefaultSpeedPwmCalibration)
{
    const auto result = speed_to_pwm(speed, calibration);
    CHECK_EQ(result.pulse_us, expected_pwm);
    CHECK_EQ(result.status, expected_status);
}

} // namespace

HOST_TEST(speed_to_pwm_maps_default_bidirectional_range_around_neutral)
{
    check_conversion(-1.0F, 1000U, ConvertStatus::Ok);
    check_conversion(-0.5F, 1250U, ConvertStatus::Ok);
    check_conversion(0.0F, 1500U, ConvertStatus::Neutral);
    check_conversion(0.5F, 1750U, ConvertStatus::Ok);
    check_conversion(1.0F, 2000U, ConvertStatus::Ok);
}

HOST_TEST(speed_to_pwm_uses_independent_spans_and_rounds_to_nearest_microsecond)
{
    constexpr SpeedPwmCalibration asymmetric{1099U, 1520U, 1901U, false};

    check_conversion(-0.5F, 1309U, ConvertStatus::Ok, asymmetric);
    check_conversion(0.5F, 1711U, ConvertStatus::Ok, asymmetric);
}

HOST_TEST(speed_to_pwm_reverses_only_the_physical_mapping)
{
    constexpr SpeedPwmCalibration reversed{1000U, 1500U, 2000U, true};

    check_conversion(-0.5F, 1750U, ConvertStatus::Ok, reversed);
    check_conversion(0.0F, 1500U, ConvertStatus::Neutral, reversed);
    check_conversion(0.5F, 1250U, ConvertStatus::Ok, reversed);
}

HOST_TEST(speed_to_pwm_clamps_out_of_range_values_after_applying_reversal)
{
    constexpr SpeedPwmCalibration reversed{1000U, 1500U, 2000U, true};

    check_conversion(-2.0F, 1000U, ConvertStatus::SaturatedReverse);
    check_conversion(2.0F, 2000U, ConvertStatus::SaturatedForward);
    check_conversion(-2.0F, 2000U, ConvertStatus::SaturatedForward, reversed);
    check_conversion(2.0F, 1000U, ConvertStatus::SaturatedReverse, reversed);
}

HOST_TEST(speed_to_pwm_rejects_non_finite_input_and_invalid_calibration)
{
    constexpr SpeedPwmCalibration min_equals_neutral{1500U, 1500U, 2000U, false};
    constexpr SpeedPwmCalibration neutral_equals_max{1000U, 2000U, 2000U, false};
    constexpr SpeedPwmCalibration descending{2000U, 1500U, 1000U, false};

    check_conversion(std::numeric_limits<float>::quiet_NaN(),
                     0U,
                     ConvertStatus::InvalidInput);
    check_conversion(std::numeric_limits<float>::infinity(),
                     0U,
                     ConvertStatus::InvalidInput);
    check_conversion(-std::numeric_limits<float>::infinity(),
                     0U,
                     ConvertStatus::InvalidInput);
    check_conversion(0.0F, 0U, ConvertStatus::InvalidConfig, min_equals_neutral);
    check_conversion(0.0F, 0U, ConvertStatus::InvalidConfig, neutral_equals_max);
    check_conversion(0.0F, 0U, ConvertStatus::InvalidConfig, descending);
}

HOST_TEST(speed_frame_to_pwm_converts_six_channels_and_clears_invalid_outputs)
{
    const MotorSpeedFrame input{{-1.0F,
                                 -0.5F,
                                 0.0F,
                                 0.5F,
                                 2.0F,
                                 std::numeric_limits<float>::quiet_NaN()},
                                0xFFU};
    const SpeedPwmCalibration calibration[kMotorCount]{
        kDefaultSpeedPwmCalibration,
        kDefaultSpeedPwmCalibration,
        kDefaultSpeedPwmCalibration,
        SpeedPwmCalibration{1000U, 1500U, 2000U, true},
        kDefaultSpeedPwmCalibration,
        kDefaultSpeedPwmCalibration,
    };
    MotorPwmFrame output{};

    speed_frame_to_pwm(input, calibration, output);

    CHECK_EQ(output.valid_mask, 0x1FU);
    CHECK_EQ(output.pulse_us[0], 1000U);
    CHECK_EQ(output.pulse_us[1], 1250U);
    CHECK_EQ(output.pulse_us[2], 1500U);
    CHECK_EQ(output.pulse_us[3], 1250U);
    CHECK_EQ(output.pulse_us[4], 2000U);
    CHECK_EQ(output.pulse_us[5], 0U);
    CHECK_EQ(output.status[0], ConvertStatus::Ok);
    CHECK_EQ(output.status[1], ConvertStatus::Ok);
    CHECK_EQ(output.status[2], ConvertStatus::Neutral);
    CHECK_EQ(output.status[3], ConvertStatus::Ok);
    CHECK_EQ(output.status[4], ConvertStatus::SaturatedForward);
    CHECK_EQ(output.status[5], ConvertStatus::InvalidInput);
}

HOST_TEST(speed_frame_to_pwm_marks_uncommanded_and_badly_configured_channels_invalid)
{
    const MotorSpeedFrame input{{0.25F, 0.25F, 0.25F, 0.25F, 0.25F, 0.25F},
                                0x15U};
    const SpeedPwmCalibration calibration[kMotorCount]{
        kDefaultSpeedPwmCalibration,
        kDefaultSpeedPwmCalibration,
        SpeedPwmCalibration{1500U, 1500U, 2000U, false},
        kDefaultSpeedPwmCalibration,
        kDefaultSpeedPwmCalibration,
        kDefaultSpeedPwmCalibration,
    };
    MotorPwmFrame output{{999U, 999U, 999U, 999U, 999U, 999U},
                         {ConvertStatus::Ok,
                          ConvertStatus::Ok,
                          ConvertStatus::Ok,
                          ConvertStatus::Ok,
                          ConvertStatus::Ok,
                          ConvertStatus::Ok},
                         0xFFU};

    speed_frame_to_pwm(input, calibration, output);

    CHECK_EQ(output.valid_mask, 0x11U);
    CHECK_EQ(output.pulse_us[0], 1625U);
    CHECK_EQ(output.status[0], ConvertStatus::Ok);
    CHECK_EQ(output.pulse_us[1], 0U);
    CHECK_EQ(output.status[1], ConvertStatus::NotCommanded);
    CHECK_EQ(output.pulse_us[2], 0U);
    CHECK_EQ(output.status[2], ConvertStatus::InvalidConfig);
    CHECK_EQ(output.pulse_us[3], 0U);
    CHECK_EQ(output.status[3], ConvertStatus::NotCommanded);
    CHECK_EQ(output.pulse_us[4], 1625U);
    CHECK_EQ(output.status[4], ConvertStatus::Ok);
    CHECK_EQ(output.pulse_us[5], 0U);
    CHECK_EQ(output.status[5], ConvertStatus::NotCommanded);
}

HOST_TEST(speed_frame_to_pwm_uses_status_not_zero_pulse_as_the_validity_source)
{
    const MotorSpeedFrame input{{-1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}, 0x01U};
    const SpeedPwmCalibration calibration[kMotorCount]{
        SpeedPwmCalibration{0U, 1500U, 2000U, false},
        kDefaultSpeedPwmCalibration,
        kDefaultSpeedPwmCalibration,
        kDefaultSpeedPwmCalibration,
        kDefaultSpeedPwmCalibration,
        kDefaultSpeedPwmCalibration,
    };
    MotorPwmFrame output{};

    speed_frame_to_pwm(input, calibration, output);

    CHECK_EQ(output.pulse_us[0], 0U);
    CHECK_EQ(output.status[0], ConvertStatus::Ok);
    CHECK_EQ(output.valid_mask, 0x01U);
}
