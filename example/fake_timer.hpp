#pragma once

class FakeTimer {
 public:
  FakeTimer() = default;

  void reset() {}

  void start() {}

  void stop() {}

  inline double get() const { return elapsedSeconds(); }

  double elapsedSeconds() const { return 0; }

  friend std::ostream& operator<<(std::ostream& os, const FakeTimer& t) {
    os << t.elapsedSeconds();
    return os;
  }

 private:
};
