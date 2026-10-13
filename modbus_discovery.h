#pragma once
// Modbus discovery (RTU and ASCII framing) for a device with an unknown register map.
// Discovery only sends functions 01-04 (reads). The one exception is start_write(): a single-register
// write (FC06), refused unless writes_enabled is set and write_allowed() accepts the register and value.
// Non-blocking: tick() is called from an ESPHome interval and runs one step of a state machine.
//
// Jobs:
//   scan        find slave addresses (FC03, one register) at the current serial settings
//   scan all    same, trying common baud / frame format / RTU-ASCII combinations until something answers
//   dump        read a register range with the selected functions and log every readable value
//   watch       repeat the dump and log only values that change
// Between jobs it polls the registers added with add_poll() (for Home Assistant sensors), one register per
// request, and otherwise listens and logs any unsolicited traffic (is something else a master on this bus?).
// A dump or watch that reads a polled register (FC03) also feeds its callback, so a watch that covers the
// polled registers keeps the sensors updating.

#include <algorithm>
#include <cstdio>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "esphome/components/uart/uart_component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace mbdisc {

using esphome::uart::UARTComponent;
using esphome::uart::UARTParityOptions;
using esphome::uart::UART_CONFIG_PARITY_EVEN;
using esphome::uart::UART_CONFIG_PARITY_NONE;
using esphome::uart::UART_CONFIG_PARITY_ODD;

static const char *const TAG = "mbdisc";
static const uint16_t BLOCK = 16;  // registers per request in a dump; failed blocks are retried one by one
static const uint32_t GAP_MS = 20;  // pause between frames, generous for slow slaves

// ESPHome's generated main.cpp defines millis() as a macro, so don't spell it qualified here.
inline uint32_t now_ms() { return millis(); }

// Frame formats, in the order of the "Modbus Frame" select in the YAML.
struct Format {
  uint8_t data_bits;
  UARTParityOptions parity;
  uint8_t stop_bits;
};
static const Format FORMATS[] = {
    {8, UART_CONFIG_PARITY_NONE, 1}, {8, UART_CONFIG_PARITY_EVEN, 1}, {8, UART_CONFIG_PARITY_ODD, 1},
    {7, UART_CONFIG_PARITY_EVEN, 1}, {7, UART_CONFIG_PARITY_ODD, 1},  {7, UART_CONFIG_PARITY_NONE, 2},
    {8, UART_CONFIG_PARITY_NONE, 2},
};
static const size_t F_8N1 = 0, F_8E1 = 1, F_8O1 = 2, F_7E1 = 3, F_7O1 = 4, F_7N2 = 5;

struct Serial {
  uint32_t baud;
  size_t format;  // index into FORMATS
  bool ascii;
  bool operator==(const Serial &o) const {
    return this->baud == o.baud && this->format == o.format && this->ascii == o.ascii;
  }
  const Format &fmt() const { return FORMATS[this->format]; }
  std::string name() const {
    const Format &f = this->fmt();
    char s[32];
    snprintf(s, sizeof(s), "%u %u%c%u %s", (unsigned) this->baud, f.data_bits,
             f.parity == UART_CONFIG_PARITY_EVEN ? 'E' : f.parity == UART_CONFIG_PARITY_ODD ? 'O' : 'N', f.stop_bits,
             this->ascii ? "ASCII" : "RTU");
    return s;
  }
  // Milliseconds on the wire for n characters.
  uint32_t chars_ms(size_t n) const {
    const Format &f = this->fmt();
    uint32_t bits = 1 + f.data_bits + (f.parity != UART_CONFIG_PARITY_NONE ? 1 : 0) + f.stop_bits;
    return (uint32_t) (n * bits * 1000 / this->baud) + 1;
  }
};

inline const char *exception_name(uint8_t code) {
  switch (code) {
    case 1: return "illegal function";
    case 2: return "illegal data address";
    case 3: return "illegal data value";
    case 4: return "slave device failure";
    case 6: return "slave busy";
    default: return "unknown";
  }
}

inline uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

inline std::string hex(const std::vector<uint8_t> &v, size_t max = 64) {
  std::string s;
  char b[4];
  for (size_t i = 0; i < v.size() && i < max; i++) {
    snprintf(b, sizeof(b), "%02X ", v[i]);
    s += b;
  }
  if (v.size() > max) s += "...";
  return s;
}

// Printable form of received bytes, for ASCII frames.
inline std::string text(const std::vector<uint8_t> &v, size_t max = 80) {
  std::string s;
  for (size_t i = 0; i < v.size() && i < max; i++) {
    uint8_t c = v[i] & 0x7F;
    if (c == '\r') {
      s += "\\r";
    } else if (c == '\n') {
      s += "\\n";
    } else {
      s += (c >= 0x20 && c < 0x7F) ? (char) c : '.';
    }
  }
  return s;
}

inline int hex_digit(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

class Discovery {
 public:
  UARTComponent *uart{nullptr};

  // Settings, written by the HA entities (see sync_settings in the YAML).
  uint8_t slave{1};
  uint16_t start{0};
  uint16_t count{100};
  uint8_t scan_last{247};
  uint32_t timeout_ms{150};
  uint32_t watch_pause_ms{2000};
  std::vector<uint8_t> functions{3, 4};

  // Called when a scan finds a slave, so the YAML can update the HA entities. Takes a copy: updating the
  // entities re-syncs this engine's settings while the callback runs.
  std::function<void(Serial serial, uint8_t addr)> on_found;

  std::string status{"idle"};

  // Periodic polling of known registers (FC03, one per request) while no discovery job runs.
  // The callback gets ok = false when the register could not be read.
  using PollCallback = std::function<void(bool ok, uint16_t raw)>;
  uint32_t poll_interval_ms{30000};
  void add_poll(uint16_t reg, PollCallback cb) { this->polls_.push_back({reg, std::move(cb)}); }

  // Guarded single-register write. Both must allow it; by default nothing is allowed.
  bool writes_enabled{false};
  std::function<bool(uint16_t reg, uint16_t value)> write_allowed = [](uint16_t, uint16_t) { return false; };

  // Writes one register with FC06, then reads it back with FC03. Returns false when refused.
  bool start_write(uint16_t reg, uint16_t value) {
    char s[96];
    if (!this->writes_enabled || !this->write_allowed(reg, value)) {
      snprintf(s, sizeof(s), "write refused: register %u = %u (%s)", reg, value,
               this->writes_enabled ? "not in allowlist" : "writes disabled");
      this->status = s;
      ESP_LOGW(TAG, "%s", s);
      return false;
    }
    this->stop_job_();
    this->job_ = WRITE;
    this->write_reg_ = reg;
    this->write_value_ = value;
    this->write_step_ = 0;
    ESP_LOGI(TAG, "Write: slave %u register %u = %u (FC06), then read back", this->slave, reg, value);
    return true;
  }

  const Serial &serial() const { return this->serial_; }
  void set_serial(const Serial &s) {
    if (s == this->serial_) return;
    this->serial_ = s;
    this->serial_dirty_ = true;
  }

  bool busy() const { return this->job_ == SCAN || this->job_ == DUMP || this->job_ == WRITE; }
  bool watching() const { return this->job_ == DUMP && this->watch_; }

  void start_scan(bool all) {
    this->stop_job_();
    this->job_ = SCAN;
    this->scan_all_ = all;
    this->hits_ = 0;
    this->user_serial_ = this->serial_;
    this->combos_.clear();
    if (all) {
      // Most likely first: per baud rate RTU 8N1/8E1 and ASCII 7E1/8N1/7N2, then the odd-parity variants.
      static const uint32_t bauds[] = {9600, 19200, 38400, 4800, 2400, 57600, 115200};
      for (uint32_t b : bauds) {
        this->combos_.push_back({b, F_8N1, false});
        this->combos_.push_back({b, F_8E1, false});
        this->combos_.push_back({b, F_7E1, true});
        this->combos_.push_back({b, F_8N1, true});
        this->combos_.push_back({b, F_7N2, true});
      }
      for (uint32_t b : bauds) {
        this->combos_.push_back({b, F_8O1, false});
        this->combos_.push_back({b, F_7O1, true});
      }
    } else {
      this->combos_.push_back(this->serial_);
    }
    this->combo_ = 0;
    this->apply_combo_();
    ESP_LOGI(TAG, "Scan: addresses 1-%u, FC03 register %u, %s", this->scan_last, this->start,
             all ? "all serial settings (RTU and ASCII), stops at the first that answers" : "current serial settings");
  }

  void start_dump(bool watch) {
    this->stop_job_();
    if (this->functions.empty() || this->count == 0) return;
    if ((uint32_t) this->start + this->count > 65536) this->count = 65536 - this->start;
    this->job_ = DUMP;
    this->watch_ = watch;
    this->pass_ = 0;
    this->values_.clear();
    this->begin_pass_();
    ESP_LOGI(TAG, "%s: slave %u, registers %u-%u, %s, %u function(s)", watch ? "Watch" : "Dump", this->slave,
             this->start, this->start + this->count - 1, this->serial_.name().c_str(),
             (unsigned) this->functions.size());
  }

  void stop() {
    if (!this->busy()) return;
    this->stop_job_();
    this->status = "stopped";
    ESP_LOGI(TAG, "Stopped");
  }

  void tick() {
    if (this->uart == nullptr) return;
    uint32_t now = now_ms();
    if (this->phase_ == WAIT) {
      this->poll_reply_(now);
      return;
    }
    if (this->serial_dirty_) this->apply_serial_();
    if (this->job_ == NONE) {
      if (this->polls_.empty() || (int32_t) (now - this->next_poll_) < 0) {
        this->listen_(now);
        return;
      }
      this->job_ = POLL;
      this->poll_idx_ = 0;
    }
    if ((int32_t) (now - this->next_send_) < 0) return;

    Req r;
    Next next = this->job_ == SCAN    ? this->next_scan_(r)
                : this->job_ == DUMP  ? this->next_dump_(r)
                : this->job_ == WRITE ? this->next_write_(r)
                                      : this->next_poll_req_(r);
    switch (next) {
      case SEND:
        this->send_(r);
        break;
      case LATER:
        break;
      case DONE:
        this->job_ = NONE;
        break;
    }
  }

 protected:
  enum Job { NONE, SCAN, DUMP, POLL, WRITE };
  enum Phase { IDLE, WAIT };
  enum Result { OK, EXCEPTION, TIMEOUT, BAD };
  enum Next { SEND, LATER, DONE };
  struct Req {
    uint8_t addr;
    uint8_t fc;
    uint16_t reg;
    uint16_t cnt;
  };

  Job job_{NONE};
  Phase phase_{IDLE};
  Serial serial_{9600, F_8N1, false};
  bool serial_dirty_{true};

  // Transaction. frame_ is the decoded reply (RTU: with CRC, ASCII: with LRC), buf_ the raw bytes.
  Req cur_{};
  std::vector<uint8_t> req_, buf_, frame_;
  uint32_t deadline_{0}, last_rx_{0}, next_send_{0};

  // Scan.
  bool scan_all_{false};
  std::vector<Serial> combos_;
  size_t combo_{0};
  uint16_t addr_{1};
  int hits_{0};
  Serial user_serial_{9600, F_8N1, false};

  // Dump / watch.
  bool watch_{false};
  size_t fn_idx_{0};
  uint32_t reg_{0};
  std::vector<uint16_t> singles_;
  bool cur_single_{false};
  uint32_t pass_{0};
  int readable_{0}, changes_{0}, timeouts_{0};
  std::map<uint32_t, uint16_t> values_;  // key: fc << 16 | register
  std::map<uint8_t, int> exceptions_;    // exception code -> count, per pass
  // Watch passes after the baseline only read the registers that answered, grouped into runs.
  std::vector<Req> runs_;
  size_t run_idx_{0};

  // Write.
  uint16_t write_reg_{0}, write_value_{0};
  int write_step_{0};
  const char *write_why_{nullptr};  // refusal reason of the FC06 step, nullptr when accepted

  // Polling.
  struct Poll {
    uint16_t reg;
    PollCallback cb;
  };
  std::vector<Poll> polls_;
  size_t poll_idx_{0};
  uint32_t next_poll_{0};

  // Idle listening.
  std::vector<uint8_t> idle_buf_;

  // --- serial ---

  void apply_serial_() {
    this->serial_dirty_ = false;
    const Format &f = this->serial_.fmt();
    this->uart->set_baud_rate(this->serial_.baud);
    this->uart->set_data_bits(f.data_bits);
    this->uart->set_parity(f.parity);
    this->uart->set_stop_bits(f.stop_bits);
    this->uart->load_settings(false);
    this->drain_();
    this->next_send_ = now_ms() + 50;
    ESP_LOGI(TAG, "Serial: %s", this->serial_.name().c_str());
  }

  void drain_() {
    uint8_t b;
    while (this->uart->available() && this->uart->read_byte(&b)) {
    }
  }

  size_t data_len_() const {
    return this->cur_.fc <= 2 ? (this->cur_.cnt + 7) / 8 : this->cur_.cnt * 2;
  }
  // Reply length in characters on the wire. FC06 replies echo the request (address, function, register, value).
  size_t expected_chars_() const {
    if (this->cur_.fc == 6) return this->serial_.ascii ? 1 + 2 * 7 + 2 : 8;
    return this->serial_.ascii ? 1 + 2 * (3 + this->data_len_() + 1) + 2 : 5 + this->data_len_();
  }

  void send_(const Req &r) {
    this->cur_ = r;
    std::vector<uint8_t> pdu = {r.addr, r.fc, (uint8_t) (r.reg >> 8), (uint8_t) r.reg, (uint8_t) (r.cnt >> 8),
                                (uint8_t) r.cnt};
    if (this->serial_.ascii) {
      uint8_t lrc = 0;
      for (uint8_t b : pdu) lrc += b;
      pdu.push_back((uint8_t) -lrc);
      static const char digits[] = "0123456789ABCDEF";
      this->req_ = {':'};
      for (uint8_t b : pdu) {
        this->req_.push_back(digits[b >> 4]);
        this->req_.push_back(digits[b & 0x0F]);
      }
      this->req_.push_back('\r');
      this->req_.push_back('\n');
    } else {
      uint16_t crc = crc16(pdu.data(), pdu.size());
      pdu.push_back(crc & 0xFF);
      pdu.push_back(crc >> 8);
      this->req_ = pdu;
    }
    this->drain_();
    this->buf_.clear();
    this->uart->write_array(this->req_.data(), this->req_.size());
    this->uart->flush();
    uint32_t now = now_ms();
    this->last_rx_ = now;
    this->deadline_ = now + this->timeout_ms + this->serial_.chars_ms(this->expected_chars_());
    this->phase_ = WAIT;
  }

  void poll_reply_(uint32_t now) {
    uint8_t b;
    while (this->uart->available() && this->uart->read_byte(&b)) {
      this->buf_.push_back(b);
      this->last_rx_ = now;
    }
    if (this->serial_.ascii ? this->poll_ascii_() : this->poll_rtu_(now)) return;
    if ((int32_t) (now - this->deadline_) > 0) this->finish_(this->buf_.empty() ? TIMEOUT : BAD, this->buf_);
  }

  // RTU: finishes on a complete frame or on silence after a partial one. Returns true when finished.
  bool poll_rtu_(uint32_t now) {
    // Drop our own request if the transceiver echoes it (not for FC06, whose reply equals the request).
    if (this->cur_.fc != 6 && this->buf_.size() >= this->req_.size() &&
        std::equal(this->req_.begin(), this->req_.end(), this->buf_.begin()))
      this->buf_.erase(this->buf_.begin(), this->buf_.begin() + this->req_.size());

    size_t need = 0;
    if (this->buf_.size() >= 2 && this->buf_[1] == (this->cur_.fc | 0x80)) {
      need = 5;
    } else if (this->buf_.size() >= 2 && this->cur_.fc == 6 && this->buf_[1] == 6) {
      need = 8;
    } else if (this->buf_.size() >= 3 && this->buf_[1] == this->cur_.fc) {
      need = 5 + this->buf_[2];
    }
    if (need > 0 && this->buf_.size() >= need) {
      std::vector<uint8_t> frame(this->buf_.begin(), this->buf_.begin() + need);
      uint16_t crc = crc16(frame.data(), need - 2);
      bool crc_ok = frame[need - 2] == (crc & 0xFF) && frame[need - 1] == (crc >> 8);
      bool echo_prefix = this->buf_.size() < this->req_.size() &&
                         std::equal(this->buf_.begin(), this->buf_.end(), this->req_.begin());
      if (crc_ok) {
        this->finish_frame_(frame, 2);
        return true;
      }
      if (!echo_prefix) {
        this->finish_(BAD, this->buf_);
        return true;
      }
    }
    if (!this->buf_.empty() && now - this->last_rx_ > 20 + this->serial_.chars_ms(4)) {
      this->finish_(BAD, this->buf_);
      return true;
    }
    return false;
  }

  // ASCII: ':' + hex + LRC + CR LF. Finishes on a complete line. Returns true when finished.
  bool poll_ascii_() {
    for (;;) {
      auto colon = std::find(this->buf_.begin(), this->buf_.end(), (uint8_t) ':');
      if (colon == this->buf_.end()) return false;
      auto lf = std::find(colon, this->buf_.end(), (uint8_t) '\n');
      if (lf == this->buf_.end()) return false;
      std::vector<uint8_t> line(colon, lf + 1);
      if (line == this->req_ && this->cur_.fc != 6) {  // echo of our request
        this->buf_.erase(this->buf_.begin(), lf + 1);
        continue;
      }
      std::vector<uint8_t> frame;
      // Hex payload is line[1 .. end-1]: at least address, function, one data byte and the LRC.
      size_t end = line.size() - 1;  // the LF
      if (end > 1 && line[end - 1] == '\r') end--;
      bool ok = (end - 1) % 2 == 0 && end - 1 >= 6;
      uint8_t sum = 0;
      for (size_t i = 1; ok && i + 1 < end; i += 2) {
        int hi = hex_digit(line[i]), lo = hex_digit(line[i + 1]);
        if (hi < 0 || lo < 0) {
          ok = false;
          break;
        }
        frame.push_back((uint8_t) (hi << 4 | lo));
        sum += frame.back();
      }
      if (!ok || sum != 0) {
        this->finish_(BAD, this->buf_);
      } else {
        this->finish_frame_(frame, 1);
      }
      return true;
    }
  }

  // A frame with a valid checksum (trailer = checksum length): check address, function and length.
  void finish_frame_(const std::vector<uint8_t> &frame, size_t trailer) {
    if (frame.size() < 3 + trailer || frame[0] != this->cur_.addr) {
      this->finish_(BAD, frame);
    } else if (frame[1] == (this->cur_.fc | 0x80)) {
      this->finish_(EXCEPTION, frame);
    } else if (this->cur_.fc == 6) {
      bool echo = frame.size() == 6 + trailer && frame[1] == 6 && ((frame[2] << 8) | frame[3]) == this->cur_.reg &&
                  ((frame[4] << 8) | frame[5]) == this->cur_.cnt;
      this->finish_(echo ? OK : BAD, frame);
    } else if (frame[1] == this->cur_.fc && frame[2] == this->data_len_() &&
               frame.size() == 3 + this->data_len_() + trailer) {
      this->finish_(OK, frame);
    } else {
      this->finish_(BAD, frame);
    }
  }

  void finish_(Result res, const std::vector<uint8_t> &frame) {
    this->phase_ = IDLE;
    this->frame_ = frame;
    this->next_send_ = now_ms() + GAP_MS + this->serial_.chars_ms(4);
    if (this->job_ == SCAN) {
      this->scan_result_(res);
    } else if (this->job_ == DUMP) {
      this->dump_result_(res);
    } else if (this->job_ == WRITE) {
      this->write_result_(res);
    } else if (this->job_ == POLL) {
      const Poll &p = this->polls_[this->poll_idx_++];
      if (res == OK) {
        p.cb(true, (this->frame_[3] << 8) | this->frame_[4]);
      } else {
        ESP_LOGW(TAG, "Poll register %u failed (%s)", p.reg,
                 res == EXCEPTION ? exception_name(this->frame_[2]) : res == TIMEOUT ? "timeout" : "invalid reply");
        p.cb(false, 0);
      }
    }
  }

  Next next_write_(Req &r) {
    if (this->write_step_ == 0) {
      r = {this->slave, 6, this->write_reg_, this->write_value_};  // cnt carries the value for FC06
    } else if (this->write_step_ == 1) {
      r = {this->slave, 3, this->write_reg_, 1};
    } else {
      this->next_poll_ = now_ms();  // refresh the sensors right away
      return DONE;
    }
    return SEND;
  }

  void write_result_(Result res) {
    const std::vector<uint8_t> &f = this->frame_;
    const char *why = res == EXCEPTION ? exception_name(f[2]) : res == TIMEOUT ? "timeout" : "invalid reply";
    if (this->write_step_ == 0) {
      this->write_why_ = res == OK ? nullptr : why;
      if (res == OK) {
        ESP_LOGI(TAG, "Write register %u = %u: accepted", this->write_reg_, this->write_value_);
      } else {
        ESP_LOGW(TAG, "Write register %u = %u: refused (%s) %s", this->write_reg_, this->write_value_, why,
                 this->raw_(f).c_str());
      }
    } else if (this->write_step_ == 1) {
      char back[32];
      if (res == OK) {
        uint16_t v = (f[3] << 8) | f[4];
        snprintf(back, sizeof(back), "read back %u", v);
        ESP_LOGI(TAG, "Read back register %u = %u (%s)", this->write_reg_, v,
                 v == this->write_value_ ? "as written" : "differs");
      } else {
        snprintf(back, sizeof(back), "read back failed");
        ESP_LOGW(TAG, "Read back register %u failed (%s)", this->write_reg_, why);
      }
      char s[96];
      snprintf(s, sizeof(s), "write register %u = %u: %s%s, %s", this->write_reg_, this->write_value_,
               this->write_why_ == nullptr ? "accepted" : "refused: ", this->write_why_ == nullptr ? "" : this->write_why_,
               back);
      this->status = s;
    }
    this->write_step_++;
  }

  Next next_poll_req_(Req &r) {
    if (this->poll_idx_ >= this->polls_.size()) {
      this->next_poll_ = now_ms() + this->poll_interval_ms;
      return DONE;
    }
    r = {this->slave, 3, this->polls_[this->poll_idx_].reg, 1};
    return SEND;
  }

  // Raw reply for the log: hex, plus the text for ASCII.
  std::string raw_(const std::vector<uint8_t> &v) const {
    return this->serial_.ascii ? hex(v) + " \"" + text(v) + "\"" : hex(v);
  }

  void stop_job_() {
    if (this->job_ == SCAN && this->scan_all_ && this->hits_ == 0) this->set_serial(this->user_serial_);
    this->job_ = NONE;
    this->phase_ = IDLE;
  }

  void listen_(uint32_t now) {
    uint8_t b;
    while (this->uart->available() && this->uart->read_byte(&b)) {
      if (this->idle_buf_.size() < 256) this->idle_buf_.push_back(b);
      this->last_rx_ = now;
    }
    if (!this->idle_buf_.empty() && now - this->last_rx_ > 20 + this->serial_.chars_ms(4)) {
      ESP_LOGI(TAG, "Unsolicited %u bytes at %s: %s", (unsigned) this->idle_buf_.size(),
               this->serial_.name().c_str(), (hex(this->idle_buf_) + " \"" + text(this->idle_buf_) + "\"").c_str());
      this->idle_buf_.clear();
    }
  }

  // --- scan ---

  void apply_combo_() {
    this->set_serial(this->combos_[this->combo_]);
    this->addr_ = 1;
  }

  Next next_scan_(Req &r) {
    if (this->addr_ > this->scan_last) {
      bool more = this->scan_all_ && this->hits_ == 0 && this->combo_ + 1 < this->combos_.size();
      if (!more) {
        this->finish_scan_();
        return DONE;
      }
      this->combo_++;
      this->apply_combo_();
      return LATER;
    }
    char s[96];
    snprintf(s, sizeof(s), "scan %s: address %u/%u, found %d", this->serial_.name().c_str(), this->addr_,
             this->scan_last, this->hits_);
    this->status = s;
    r = {(uint8_t) this->addr_, 3, this->start, 1};
    return SEND;
  }

  void scan_result_(Result res) {
    uint8_t addr = this->addr_++;
    const std::vector<uint8_t> &f = this->frame_;
    if (res == OK || res == EXCEPTION) {
      this->hits_++;
      if (res == OK) {
        ESP_LOGI(TAG, "FOUND slave %u at %s, register %u = %u (reply %s)", addr, this->serial_.name().c_str(),
                 this->start, (f[3] << 8) | f[4], hex(f).c_str());
      } else {
        ESP_LOGI(TAG, "FOUND slave %u at %s, exception %u (%s) for register %u", addr, this->serial_.name().c_str(),
                 f[2], exception_name(f[2]), this->start);
      }
      if (this->hits_ == 1) {
        this->slave = addr;
        if (this->on_found) this->on_found(this->serial_, addr);
      }
    } else if (res == BAD) {
      ESP_LOGW(TAG, "Address %u at %s: invalid reply %s (something answers; wrong serial settings?)", addr,
               this->serial_.name().c_str(), this->raw_(f).c_str());
    }
  }

  void finish_scan_() {
    char s[96];
    if (this->hits_ > 0) {
      snprintf(s, sizeof(s), "scan done: %d slave(s), first %u at %s", this->hits_, this->slave,
               this->serial_.name().c_str());
    } else {
      snprintf(s, sizeof(s), "scan done: no reply");
      if (this->scan_all_) this->set_serial(this->user_serial_);
    }
    this->status = s;
    ESP_LOGI(TAG, "%s", s);
  }

  // --- dump / watch ---

  void begin_pass_() {
    this->fn_idx_ = 0;
    this->reg_ = this->start;
    this->singles_.clear();
    this->readable_ = 0;
    this->changes_ = 0;
    this->timeouts_ = 0;
    this->exceptions_.clear();
    this->run_idx_ = 0;
  }

  bool cached_pass_() const { return this->watch_ && this->pass_ > 0; }

  // Groups the registers that answered in the baseline into contiguous runs of at most BLOCK.
  void build_runs_() {
    this->runs_.clear();
    for (const auto &kv : this->values_) {
      uint8_t fc = kv.first >> 16;
      uint16_t reg = kv.first & 0xFFFF;
      if (!this->runs_.empty()) {
        Req &last = this->runs_.back();
        if (last.fc == fc && last.reg + last.cnt == reg && last.cnt < BLOCK) {
          last.cnt++;
          continue;
        }
      }
      this->runs_.push_back({0, fc, reg, 1});
    }
    ESP_LOGI(TAG, "Watch: next passes read only the %u answering registers in %u requests",
             (unsigned) this->values_.size(), (unsigned) this->runs_.size());
  }

  Next next_dump_(Req &r) {
    uint32_t end = (uint32_t) this->start + this->count;
    if (this->cached_pass_()) {
      if (this->run_idx_ >= this->runs_.size()) return this->next_pass_();
      r = this->runs_[this->run_idx_];
      r.addr = this->slave;
      this->cur_single_ = false;
      char s[96];
      snprintf(s, sizeof(s), "watch pass %u: FC%02u register %u (%u registers), %d changes", this->pass_ + 1, r.fc,
               r.reg, (unsigned) this->values_.size(), this->changes_);
      this->status = s;
      return SEND;
    }
    if (this->singles_.empty() && this->reg_ >= end) {
      this->fn_idx_++;
      this->reg_ = this->start;
    }
    if (this->fn_idx_ >= this->functions.size()) return this->next_pass_();
    uint8_t fc = this->functions[this->fn_idx_];
    this->cur_single_ = !this->singles_.empty();
    if (this->cur_single_) {
      r = {this->slave, fc, this->singles_.front(), 1};
    } else {
      r = {this->slave, fc, (uint16_t) this->reg_, (uint16_t) std::min<uint32_t>(BLOCK, end - this->reg_)};
    }
    char s[96];
    snprintf(s, sizeof(s), "%s pass %u: FC%02u register %u (%u-%u), %d readable, %d changes",
             this->watch_ ? "watch" : "dump", this->pass_ + 1, fc, r.reg, this->start, end - 1, this->readable_,
             this->changes_);
    this->status = s;
    return SEND;
  }

  Next next_pass_() {
    this->end_pass_();
    if (!this->watch_) return DONE;
    if (this->pass_ == 0) this->build_runs_();
    this->pass_++;
    this->begin_pass_();
    this->next_send_ = now_ms() + this->watch_pause_ms;
    return LATER;
  }

  void dump_result_(Result res) {
    const std::vector<uint8_t> &f = this->frame_;
    if (res == TIMEOUT) this->timeouts_++;
    // Count exception codes once per register (a failed block is retried per register, count those).
    if (res == EXCEPTION && this->cur_.cnt == 1) this->exceptions_[f[2]]++;
    if (this->cached_pass_()) {
      // Some slaves refuse multi-register reads across internal table boundaries even when every
      // register answers on its own: split a refused run into single registers for good.
      if (res != OK && this->cur_.cnt > 1) {
        ESP_LOGI(TAG, "Watch: registers %u-%u refused as one read (%s), reading them one by one", this->cur_.reg,
                 this->cur_.reg + this->cur_.cnt - 1,
                 res == EXCEPTION ? exception_name(f[2]) : res == TIMEOUT ? "timeout" : "invalid reply");
        Req run = this->runs_[this->run_idx_];
        this->runs_.erase(this->runs_.begin() + this->run_idx_);
        for (uint16_t i = 0; i < run.cnt; i++)
          this->runs_.insert(this->runs_.begin() + this->run_idx_ + i, Req{0, run.fc, (uint16_t) (run.reg + i), 1});
        return;
      }
      this->run_idx_++;
    } else if (this->cur_single_) {
      this->singles_.erase(this->singles_.begin());
    } else {
      this->reg_ += this->cur_.cnt;
      // A block can fail because one register in it is invalid: retry the registers one by one.
      if (res != OK && this->cur_.cnt > 1)
        for (uint16_t i = 0; i < this->cur_.cnt; i++) this->singles_.push_back(this->cur_.reg + i);
    }
    if (res == BAD)
      ESP_LOGW(TAG, "FC%02u register %u: invalid reply %s", this->cur_.fc, this->cur_.reg, this->raw_(f).c_str());
    if (res != OK) return;

    for (uint16_t i = 0; i < this->cur_.cnt; i++) {
      uint16_t v = this->cur_.fc <= 2 ? (f[3 + i / 8] >> (i % 8)) & 1 : (f[3 + 2 * i] << 8) | f[4 + 2 * i];
      this->record_(this->cur_.fc, this->cur_.reg + i, v);
    }
  }

  void record_(uint8_t fc, uint16_t reg, uint16_t v) {
    this->readable_++;
    if (fc == 3)
      for (const auto &p : this->polls_)
        if (p.reg == reg) p.cb(true, v);
    uint32_t key = (uint32_t) fc << 16 | reg;
    auto it = this->values_.find(key);
    if (it == this->values_.end()) {
      // First time seen: log the full value. Signed and /10 columns help spot temperatures (x0.1 degC).
      if (fc <= 2) {
        ESP_LOGI(TAG, "FC%02u %5u = %u", fc, reg, v);
      } else {
        ESP_LOGI(TAG, "FC%02u %5u (0x%04X) = %5u  0x%04X  signed %6d  /10 %7.1f", fc, reg, reg, v, v, (int16_t) v,
                 (int16_t) v / 10.0f);
      }
      this->values_[key] = v;
    } else if (it->second != v) {
      this->changes_++;
      if (fc <= 2) {
        ESP_LOGI(TAG, "CHANGE FC%02u %5u: %u -> %u", fc, reg, it->second, v);
      } else {
        ESP_LOGI(TAG, "CHANGE FC%02u %5u: %u -> %u  (signed %d -> %d, /10 %.1f -> %.1f)", fc, reg, it->second, v,
                 (int16_t) it->second, (int16_t) v, (int16_t) it->second / 10.0f, (int16_t) v / 10.0f);
      }
      it->second = v;
    }
  }

  void end_pass_() {
    char s[96];
    if (this->pass_ == 0 || !this->watch_) {
      snprintf(s, sizeof(s), "%s: %d readable, %d timeouts", this->watch_ ? "watch baseline" : "dump done",
               this->readable_, this->timeouts_);
      ESP_LOGI(TAG, "%s", s);
      for (const auto &e : this->exceptions_)
        ESP_LOGI(TAG, "  exception %u (%s): %d register(s)", e.first, exception_name(e.first), e.second);
    } else {
      snprintf(s, sizeof(s), "watch pass %u: %d readable, %d changes", this->pass_ + 1, this->readable_,
               this->changes_);
      ESP_LOGD(TAG, "%s", s);
    }
    this->status = s;
  }
};

}  // namespace mbdisc

static mbdisc::Discovery disc;
