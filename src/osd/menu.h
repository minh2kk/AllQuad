// AllQuad - OSD menu (Betaflight CMS / KK2 on-board menu): change PID, rates, filters, VTX, battery and a
// few flight settings through the goggles, with the sticks, while disarmed. No computer needed.
//
//   open:  throttle centre + yaw left + pitch up (held 1 s, disarmed, arm switch off)
//   pitch up / down   move the cursor
//   roll right / left change the value (held: repeats, faster after 1.5 s), open a page / run an action
//   "BACK" goes up a page, "EXIT" / "SAVE + EXIT" close the menu. Arming is blocked while it is open.
//
// Values go through the parameter table (paramSet): the same limits as the CLI and the configurator.
#pragma once
#include <stdint.h>

namespace osd {

class Menu {
 public:
  static constexpr int LINE = 29;  // text columns of one menu line (fits the 30-column SD canvas)
  bool isOpen() const { return open_; }
  void openMenu(uint32_t nowMs);
  void close(bool save);
  // sticks -1..1 (roll right +, pitch forward +), called at 20-100 Hz while open
  void input(float roll, float pitch, uint32_t nowMs);
  // the visible lines (title first), returns how many; maxLines = rows of the canvas
  int lines(char out[][LINE + 1], int maxLines) const;
  int page() const { return page_; }
  int cursor() const { return cur_; }
  // for tests: press roll right / left once on the current line, move the cursor
  void press(int dir);
  void move(int dir);

 private:
  int itemCount() const;
  bool open_ = false;
  int page_ = 0, cur_ = 0, top_ = 0;
  int heldDir_ = 0, heldAxis_ = -1;   // 0 pitch, 1 roll
  uint32_t heldSinceMs_ = 0, lastRepeatMs_ = 0, openedMs_ = 0;
  bool neutralSeen_ = false;          // the sticks came back to centre after the open command
};

extern Menu menu;

}  // namespace osd
