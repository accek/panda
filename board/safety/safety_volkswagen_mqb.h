#include "safety_volkswagen_common.h"

// lateral limits
const SteeringLimits VOLKSWAGEN_MQB_STEERING_LIMITS = {
  .max_steer = 300,              // 3.0 Nm (EPS side max of 3.0Nm with fault if violated)
  .max_rt_delta = 113,           // 4 max rate up * 50Hz send rate * 250000 RT interval / 1000000 = 50 ; 50 * 1.5 for safety pad = 75
  .max_rt_interval = 250000,     // 250ms between real time checks
  .max_rate_up = 6,              // 3.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
  .max_rate_down = 10,           // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
  .driver_torque_allowance = 80,
  .driver_torque_factor = 3,
  .type = TorqueDriverLimited,
};

// longitudinal limits
// acceleration in m/s2 * 1000 to avoid floating point math
const LongitudinalLimits VOLKSWAGEN_MQB_LONG_LIMITS = {
  .max_accel = 2000,
  .min_accel = -3500,
  .inactive_accel = 3010,  // VW sends one increment above the max range when inactive
  .allow_long_with_gas_override = true,  // TSK requires valid accel input when gas is pressed
};

// Allow accel control for a short time after long is disabled. This is because rejecting even
// single messages evenry now and then will ultimately cause the ACC module to fault.
const uint32_t VOLKSWAGEN_MQB_ACC_CHECKS_GRACE_PERIOD_US = 50000;  // 50ms

#define MSG_ESP_19      0x0B2   // RX from ABS, for wheel speeds
#define MSG_LH_EPS_03   0x09F   // RX from EPS, for driver steering torque
#define MSG_ESP_05      0x106   // RX from ABS, for brake switch state
#define MSG_TSK_06      0x120   // RX from ECU, for ACC status from drivetrain coordinator
#define MSG_MOTOR_20    0x121   // RX from ECU, for driver throttle input
#define MSG_ACC_06      0x122   // TX by OP, ACC control instructions to the drivetrain coordinator
#define MSG_HCA_01      0x126   // TX by OP, Heading Control Assist steering torque
#define MSG_GRA_ACC_01  0x12B   // TX by OP, ACC control buttons for cancel/resume
#define MSG_ACC_07      0x12E   // TX by OP, ACC control instructions to the drivetrain coordinator
#define MSG_ACC_02      0x30C   // TX by OP, ACC HUD data to the instrument cluster
#define MSG_ACC_04      0x324   // TX by OP, ACC HUD data to the instrument cluster
#define MSG_ACC_13      0x2A7   // TX by OP, ACC HUD data to the instrument cluster
#define MSG_MOTOR_14    0x3BE   // RX from ECU, for brake switch status
#define MSG_LDW_02      0x397   // TX by OP, Lane line recognition and text alerts

// Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
const CanMsg VOLKSWAGEN_MQB_STOCK_TX_MSGS[] = {{MSG_HCA_01, 0, 8}, {MSG_GRA_ACC_01, 0, 8}, {MSG_GRA_ACC_01, 2, 8},
                                               {MSG_LDW_02, 0, 8}, {MSG_LH_EPS_03, 2, 8}};
const CanMsg VOLKSWAGEN_MQB_LONG_TX_MSGS[] = {{MSG_HCA_01, 0, 8}, {MSG_GRA_ACC_01, 2, 8},
                                              {MSG_LDW_02, 0, 8}, {MSG_LH_EPS_03, 2, 8}, {MSG_TSK_06, 2, 8},
                                              {MSG_ACC_02, 0, 8}, {MSG_ACC_04, 0, 8}, {MSG_ACC_06, 0, 8},
                                              {MSG_ACC_07, 0, 8}, {MSG_ACC_13, 0, 8}};

RxCheck volkswagen_mqb_rx_checks[] = {
  {.msg = {{MSG_ESP_19, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{MSG_LH_EPS_03, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{MSG_ESP_05, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{MSG_TSK_06, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{MSG_MOTOR_20, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{MSG_MOTOR_14, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 10U}, { 0 }, { 0 }}},
  {.msg = {{MSG_GRA_ACC_01, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 33U}, { 0 }, { 0 }}},
  {.msg = {{MSG_ACC_02, 2, 8, .check_checksum = true, .max_counter = 15U, .frequency = 17U}, { 0 }, { 0 }}},
  {.msg = {{MSG_ACC_04, 2, 8, .check_checksum = true, .max_counter = 15U, .frequency = 17U}, { 0 }, { 0 }}},
  {.msg = {{MSG_ACC_06, 2, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{MSG_ACC_07, 2, 8, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
};

typedef struct {
  CanMsg msg;
  int8_t last_counter;
} MqbCounterCheck;

MqbCounterCheck volkswagen_mqb_long_counter_checks[] = {
  {.msg = {MSG_ACC_02, 0, 8}},
  {.msg = {MSG_ACC_04, 0, 8}},
  {.msg = {MSG_ACC_06, 0, 8}},
  {.msg = {MSG_ACC_07, 0, 8}},
  {.msg = {MSG_ACC_13, 0, 8}},
  {.msg = {MSG_TSK_06, 2, 8}},
  {.msg = {-1, 0, 0}},
};

uint8_t volkswagen_crc8_lut_8h2f[256]; // Static lookup table for CRC8 poly 0x2F, aka 8H2F/AUTOSAR
bool volkswagen_mqb_brake_pedal_switch = false;
bool volkswagen_mqb_brake_pressure_detected = false;
uint32_t volkswagen_mqb_long_allowed_last_ts = 0;

static uint32_t volkswagen_mqb_get_checksum(const CANPacket_t *to_push) {
  return (uint8_t)GET_BYTE(to_push, 0);
}

static uint8_t volkswagen_mqb_get_counter(const CANPacket_t *to_push) {
  // MQB message counters are consistently found at LSB 8.
  return (uint8_t)GET_BYTE(to_push, 1) & 0xFU;
}

static uint32_t volkswagen_mqb_compute_crc(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);

  // This is CRC-8H2F/AUTOSAR with a twist. See the OpenDBC implementation
  // of this algorithm for a version with explanatory comments.

  uint8_t crc = 0xFFU;
  for (int i = 1; i < len; i++) {
    crc ^= (uint8_t)GET_BYTE(to_push, i);
    crc = volkswagen_crc8_lut_8h2f[crc];
  }

  uint8_t counter = volkswagen_mqb_get_counter(to_push);
  if (addr == MSG_LH_EPS_03) {
    crc ^= (uint8_t[]){0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5}[counter];
  } else if (addr == MSG_ESP_05) {
    crc ^= (uint8_t[]){0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07}[counter];
  } else if (addr == MSG_TSK_06) {
    crc ^= (uint8_t[]){0xC4,0xE2,0x4F,0xE4,0xF8,0x2F,0x56,0x81,0x9F,0xE5,0x83,0x44,0x05,0x3F,0x97,0xDF}[counter];
  } else if (addr == MSG_MOTOR_20) {
    crc ^= (uint8_t[]){0xE9,0x65,0xAE,0x6B,0x7B,0x35,0xE5,0x5F,0x4E,0xC7,0x86,0xA2,0xBB,0xDD,0xEB,0xB4}[counter];
  } else if (addr == MSG_GRA_ACC_01) {
    crc ^= (uint8_t[]){0x6A,0x38,0xB4,0x27,0x22,0xEF,0xE1,0xBB,0xF8,0x80,0x84,0x49,0xC7,0x9E,0x1E,0x2B}[counter];
  } else if (addr == MSG_ACC_02) {
    crc ^= (uint8_t[]){0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F}[counter];
  } else if (addr == MSG_ACC_04) {
    crc ^= (uint8_t[]){0x27,0x27,0x27,0x27,0x27,0x27,0x27,0x27,0x27,0x27,0x27,0x27,0x27,0x27,0x27,0x27}[counter];
  } else if (addr == MSG_ACC_06) {
    crc ^= (uint8_t[]){0x37,0x7D,0xF3,0xA9,0x18,0x46,0x6D,0x4D,0x3D,0x71,0x92,0x9C,0xE5,0x32,0x10,0xB9}[counter];
  } else if (addr == MSG_ACC_07) {
    crc ^= (uint8_t[]){0xF8,0xE5,0x97,0xC9,0xD6,0x07,0x47,0x21,0x66,0xDD,0xCF,0x6F,0xA1,0x94,0x74,0x63}[counter];
  } else {
    // Undefined CAN message, CRC check expected to fail
  }
  crc = volkswagen_crc8_lut_8h2f[crc];

  return (uint8_t)(crc ^ 0xFFU);
}

static bool volkswagen_mqb_longitudinal_accel_checks(int desired_accel, const LongitudinalLimits limits) {
  if (!longitudinal_accel_checks(desired_accel, limits)) {
    return false;
  }
  if (get_ts_elapsed(microsecond_timer_get(), volkswagen_mqb_long_allowed_last_ts) < VOLKSWAGEN_MQB_ACC_CHECKS_GRACE_PERIOD_US) {
    return max_limit_check(desired_accel, limits.max_accel, limits.min_accel);
  }
  return true;
}

static safety_config volkswagen_mqb_init(uint16_t param) {
  UNUSED(param);

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;
  volkswagen_stock_acc_engaged = false;
  volkswagen_mqb_brake_pedal_switch = false;
  volkswagen_mqb_brake_pressure_detected = false;
  volkswagen_mqb_long_allowed_last_ts = 0;

  for (MqbCounterCheck* check = &volkswagen_mqb_long_counter_checks[0]; check->msg.addr != -1; check++) {
    check->last_counter = -1;
  }

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#endif

  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);
  return volkswagen_longitudinal ? BUILD_SAFETY_CFG(volkswagen_mqb_rx_checks, VOLKSWAGEN_MQB_LONG_TX_MSGS) : \
                                   BUILD_SAFETY_CFG(volkswagen_mqb_rx_checks, VOLKSWAGEN_MQB_STOCK_TX_MSGS);
}

static void volkswagen_mqb_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == 0) {

    // Update in-motion state by sampling wheel speeds
    if (addr == MSG_ESP_19) {
      // sum 4 wheel speeds
      int speed = 0;
      for (uint8_t i = 0U; i < 8U; i += 2U) {
        int wheel_speed = GET_BYTE(to_push, i) | (GET_BYTE(to_push, i + 1U) << 8);
        speed += wheel_speed;
      }
      // Check all wheel speeds for any movement
      vehicle_moving = speed > 0;
    }

    // Update driver input torque samples
    // Signal: LH_EPS_03.EPS_Lenkmoment (absolute torque)
    // Signal: LH_EPS_03.EPS_VZ_Lenkmoment (direction)
    if (addr == MSG_LH_EPS_03) {
      int torque_driver_new = GET_BYTE(to_push, 5) | ((GET_BYTE(to_push, 6) & 0x1FU) << 8);
      int sign = (GET_BYTE(to_push, 6) & 0x80U) >> 7;
      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    if (addr == MSG_TSK_06) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Always exit controls on main switch off
      // Signal: TSK_06.TSK_Status
      int acc_status = (GET_BYTE(to_push, 3) & 0x7U);
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      if (!volkswagen_longitudinal) {
        pcm_cruise_check(cruise_engaged);
      }

      mads_acc_main_check(acc_main_on);
    }

    if (addr == MSG_GRA_ACC_01) {
      // If using openpilot longitudinal, enter controls on falling edge of Set or Resume with main switch on
      // Signal: GRA_ACC_01.GRA_Tip_Setzen
      // Signal: GRA_ACC_01.GRA_Tip_Wiederaufnahme
      if (volkswagen_longitudinal) {
        bool set_button = GET_BIT(to_push, 16U);
        bool resume_button = GET_BIT(to_push, 19U);
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
          controls_allowed_long = acc_main_on;
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
      }

      // Always exit controls on rising edge of Cancel
      // Signal: GRA_ACC_01.GRA_Abbrechen
      if (GET_BIT(to_push, 13U)) {
        controls_allowed_long = false;
      }
    }

    // Signal: Motor_20.MO_Fahrpedalrohwert_01
    if (addr == MSG_MOTOR_20) {
      gas_pressed = ((GET_BYTES(to_push, 0, 4) >> 12) & 0xFFU) != 0U;
    }

    // Signal: Motor_14.MO_Fahrer_bremst (ECU detected brake pedal switch F63)
    if (addr == MSG_MOTOR_14) {
      volkswagen_mqb_brake_pedal_switch = (GET_BYTE(to_push, 3) & 0x10U) >> 4;
    }

    // Signal: ESP_05.ESP_Fahrer_bremst (ESP detected driver brake pressure above platform specified threshold)
    if (addr == MSG_ESP_05) {
      volkswagen_mqb_brake_pressure_detected = (GET_BYTE(to_push, 3) & 0x4U) >> 2;
    }

    brake_pressed = volkswagen_mqb_brake_pedal_switch || volkswagen_mqb_brake_pressure_detected;

    generic_rx_checks((addr == MSG_HCA_01));

    if (volkswagen_longitudinal) {
      bool long_allowed = get_longitudinal_allowed(VOLKSWAGEN_MQB_LONG_LIMITS.allow_long_with_gas_override);
      uint32_t now = microsecond_timer_get();
      if (long_allowed) {
        volkswagen_mqb_long_allowed_last_ts = now;
      } else if (get_ts_elapsed(now, volkswagen_mqb_long_allowed_last_ts) > VOLKSWAGEN_MQB_ACC_CHECKS_GRACE_PERIOD_US) {
        // Prevent overflow
        volkswagen_mqb_long_allowed_last_ts = now - VOLKSWAGEN_MQB_ACC_CHECKS_GRACE_PERIOD_US - 1;
      }
    }
  }
}

static bool volkswagen_mqb_long_counter_check_and_update(const CANPacket_t *to_send, int8_t tx_bus) {
  int addr = GET_ADDR(to_send);
  for (MqbCounterCheck* check = &volkswagen_mqb_long_counter_checks[0]; check->msg.addr != -1; check++) {
    if (check->msg.addr == addr && check->msg.bus == tx_bus) {
      uint8_t counter = volkswagen_mqb_get_counter(to_send);
      if (counter != ((check->last_counter + 1) % 16) && check->last_counter != -1) {
        return false;
      } else {
        check->last_counter = counter;
      }
    }
  }
  return true;
}

static bool volkswagen_mqb_tx_hook(const CANPacket_t *to_send) {
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);
  bool tx = true;

  // TODO: consider detesting AE pass-through and allowing forwarding everything in this case

  // Safety check for HCA_01 Heading Control Assist torque
  // Signal: HCA_01.HCA_01_LM_Offset (absolute torque)
  // Signal: HCA_01.HCA_01_LM_OffSign (direction)
  if (addr == MSG_HCA_01) {
    int desired_torque = GET_BYTE(to_send, 2) | ((GET_BYTE(to_send, 3) & 0x1U) << 8);
    bool sign = GET_BIT(to_send, 31U);
    if (sign) {
      desired_torque *= -1;
    }

    bool steer_req = GET_BIT(to_send, 30U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, VOLKSWAGEN_MQB_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // Safety check for both ACC_06 and ACC_07 acceleration requests
  // To avoid floating point math, scale upward and compare to pre-scaled safety m/s2 boundaries
  if ((addr == MSG_ACC_06) || (addr == MSG_ACC_07)) {
    bool violation = false;
    int desired_accel = 0;

    if (addr == MSG_ACC_06) {
      // Signal: ACC_06.ACC_Sollbeschleunigung_02 (acceleration in m/s2, scale 0.005, offset -7.22)
      desired_accel = ((((GET_BYTE(to_send, 4) & 0x7U) << 8) | GET_BYTE(to_send, 3)) * 5U) - 7220U;
    } else {
      // Signal: ACC_07.ACC_Folgebeschl (acceleration in m/s2, scale 0.03, offset -4.6)
      //int secondary_accel = (GET_BYTE(to_send, 4) * 30U) - 4600U;
      //violation |= (secondary_accel != 3020);  // enforce always inactive (one increment above max range) at this time
      // Signal: ACC_07.ACC_Sollbeschleunigung_02 (acceleration in m/s2, scale 0.005, offset -7.22)
      desired_accel = (((GET_BYTE(to_send, 7) << 3) | ((GET_BYTE(to_send, 6) & 0xE0U) >> 5)) * 5U) - 7220U;
    }

    violation |= volkswagen_mqb_longitudinal_accel_checks(desired_accel, VOLKSWAGEN_MQB_LONG_LIMITS);

    if (violation) {
      tx = false;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if (addr == MSG_GRA_ACC_01) {
    // disallow resume and set: bits 16 and 19
    /*if ((GET_BYTE(to_send, 2) & 0x9U) != 0U) {
      tx = 0;
    }*/
    bool is_set_cruise = GET_BIT(to_send, 16U) != 0U;
    bool is_resume_cruise = GET_BIT(to_send, 19U) != 0U;
    bool is_accel_cruise = GET_BIT(to_send, 17U) != 0U;
    bool is_decel_cruise = GET_BIT(to_send, 18U) != 0U;
    bool is_cancel = GET_BIT(to_send, 13U) != 0U;
    bool allowed = volkswagen_longitudinal || (is_cancel && cruise_engaged_prev) ||
                   ((is_set_cruise || is_resume_cruise || is_accel_cruise || is_decel_cruise) && controls_allowed && controls_allowed_long);
    if (!allowed) {
      tx = false;
    }
  }

  // Do not allow injecting ACC messages if we forward them from the stock ACC module
  if (volkswagen_longitudinal && tx) {
    tx = volkswagen_mqb_long_counter_check_and_update(to_send, bus);
  }

  return tx;
}

static int volkswagen_mqb_fwd_hook(const CANPacket_t *to_push) {
  int bus_num = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);
  int bus_fwd = -1;

  switch (bus_num) {
    case 0:
      if (addr == MSG_LH_EPS_03) {
        // openpilot needs to replace apparent driver steering input torque to pacify VW Emergency Assist
        bus_fwd = -1;
      } else if (volkswagen_longitudinal && !volkswagen_stock_acc_engaged && (addr == MSG_TSK_06)) {
        // openpilot needs to replace ACC feedback from TSK so that stock ACC would not fault
        bus_fwd = -1;
      } else if (volkswagen_longitudinal && (addr == MSG_GRA_ACC_01)) {
        // openpilot needs to replace ACC control input to ensure that the stock ACC is never active, yet
        // monitoring
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN onward
        bus_fwd = 2;
      }
      break;
    case 2:
      if (volkswagen_longitudinal && addr == MSG_ACC_06) {
        int acc_status = (GET_BYTE(to_push, 7) & 0x70U) >> 4;
        volkswagen_stock_acc_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5) || (acc_status == 6);
      }
      if ((addr == MSG_HCA_01) || (addr == MSG_LDW_02)) {
        // openpilot takes over LKAS steering control and related HUD messages from the camera
        bus_fwd = -1;
      } else if (volkswagen_longitudinal && !volkswagen_stock_acc_engaged
          && ((addr == MSG_ACC_02) || (addr == MSG_ACC_04) || (addr == MSG_ACC_06) || (addr == MSG_ACC_07) || (addr == MSG_ACC_13))) {
        // openpilot takes over acceleration/braking control and related HUD messages from the stock ACC radar
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN devices to J533 gateway
        bus_fwd = 0;
      }
      break;
    default:
      // No other buses should be in use; fallback to do-not-forward
      bus_fwd = -1;
      break;
  }

  // This is mostly to update counters to avoid duplicate messages when switching from stock to OP.
  // The check should never fail.
  if (volkswagen_longitudinal && bus_fwd != -1) {
    if (!volkswagen_mqb_long_counter_check_and_update(to_push, bus_fwd)) {
      bus_fwd = -1;
    }
  }

  return bus_fwd;
}

const safety_hooks volkswagen_mqb_hooks = {
  .init = volkswagen_mqb_init,
  .rx = volkswagen_mqb_rx_hook,
  .tx = volkswagen_mqb_tx_hook,
  .fwd = volkswagen_mqb_fwd_hook,
  .get_counter = volkswagen_mqb_get_counter,
  .get_checksum = volkswagen_mqb_get_checksum,
  .compute_checksum = volkswagen_mqb_compute_crc,
};
