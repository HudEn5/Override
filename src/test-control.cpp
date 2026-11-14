#include "test-control.h"

void test_control::UpdateSelection() {
  info.request_mm = info.selected == test_motion::straight ? info.direction * settings.straight_mm : settings.before_mm;
  info.request_deg = info.selected == test_motion::straight ? 0 : info.direction *
      (info.selected == test_motion::turn ? settings.turn_deg : settings.corner_deg);
  info.maximum_rpm = info.selected == test_motion::straight ? settings.straight_rpm :
      info.selected == test_motion::turn ? settings.turn_rpm : settings.corner_rpm;
}
void test_control::ResetMotion() {
  Straight.Reset(); Turn.Reset(); Corner.Reset(); info.status = "READY"; Publish();
}
void test_control::Publish() {
  info.straight = Straight.Info(); info.turn = Turn.Info(); info.corner = Corner.Info();
  info.target_rpm = !info.running ? 0 : info.selected == test_motion::straight ? info.straight.motion.target_rpm :
      info.selected == test_motion::turn ? info.turn.target_rpm : info.corner.center_rpm;
}
void test_control::Cancel() {
  Straight.CancelDriveStraight(); Turn.CancelTurn(); Corner.CancelCorner();
  info.running = false; info.status = "CANCEL"; Publish();
}
bool test_control::Update(const manual::Input& input, manual::Output& output, bool allowed,
                          double now, const drive::straight_feedback& frame) {
  Straight.Observe(frame, allowed, now); Turn.Observe(frame, allowed, now); Corner.Observe(frame, allowed, now);
  const bool start = input.test_start && !previous_start;
  const bool next = input.test_next && !previous_next;
  const bool previous = input.test_previous && !previous_previous;
  const bool positive = input.test_positive && !previous_positive;
  const bool negative = input.test_negative && !previous_negative;
  const bool was_allowed = previous_allowed;
  previous_start = input.test_start; previous_next = input.test_next; previous_previous = input.test_previous;
  previous_positive = input.test_positive; previous_negative = input.test_negative; previous_allowed = allowed;
  const bool sticks = output.L_voltage != 0 || output.R_voltage != 0;
  if (!allowed || sticks) {
    if (info.running) Cancel();
    info.visible = false;
    return false;
  }
  info.visible = true;
  output.L_voltage = output.R_voltage = 0;
  // B 优先级最高；按住 B 不允许 A 或选择键启动动作。
  if (input.test_cancel) { Cancel(); return true; }
  if (!info.running && was_allowed) {
    if ((next || previous || positive || negative) && frame.stopped) {
      if (input.test_next != input.test_previous && (next || previous)) {
        const int delta = next ? 1 : 2;
        info.selected = static_cast<test_motion>((static_cast<int>(info.selected) + delta) % 3);
      }
      if (input.test_positive != input.test_negative && (positive || negative)) info.direction = positive ? 1 : -1;
      UpdateSelection(); ResetMotion();
    }
    const bool selecting = input.test_next || input.test_previous || input.test_positive || input.test_negative;
    if (start && !selecting) {
      if (!frame.stopped) info.status = "MOVING";
      else {
        ResetMotion();
        const bool started = info.selected == test_motion::straight ?
            Straight.StartDriveStraight(info.request_mm, info.maximum_rpm) : info.selected == test_motion::turn ?
            Turn.StartTurnRelative(info.request_deg, info.maximum_rpm) :
            Corner.StartDriveCorner(settings.before_mm, info.request_deg, settings.after_mm, info.maximum_rpm);
        info.running = started;
        if (!started) info.status = info.selected == test_motion::straight ? Straight.Info().status :
            info.selected == test_motion::turn ? Turn.Info().status : Corner.Info().status;
      }
    }
  }
  if (info.running) {
    if (info.selected == test_motion::straight) {
      Straight.UpdateDriveStraight(); info.status = Straight.Info().status;
      info.running = Straight.Info().state == drive::straight_state::running;
    } else if (info.selected == test_motion::turn) {
      Turn.UpdateTurn(); info.status = Turn.Info().status;
      info.running = Turn.Info().state == drive::turn_state::running;
    } else {
      Corner.UpdateCorner(); info.status = Corner.Info().status;
      info.running = Corner.Info().state == drive::corner_state::running;
    }
  }
  Publish();
  if (info.running) {
    output.L_voltage = info.selected == test_motion::straight ? info.straight.left_mV :
        info.selected == test_motion::turn ? info.turn.left_mV : info.corner.left_mV;
    output.R_voltage = info.selected == test_motion::straight ? info.straight.right_mV :
        info.selected == test_motion::turn ? info.turn.right_mV : info.corner.right_mV;
  }
  return true;
}
