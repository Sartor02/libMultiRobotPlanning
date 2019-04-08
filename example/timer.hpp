#pragma once

#include <chrono>
#include <iostream>

class Timer {
 public:
  Timer()
      : start_(std::chrono::high_resolution_clock::now()),
        end_(std::chrono::high_resolution_clock::now()), total_(0) {}

  void reset() { 
    total_ = 0;
    start();
  }
  
  void start() {
    start_ = std::chrono::high_resolution_clock::now(); 
  }

  void stop() { 
    end_ = std::chrono::high_resolution_clock::now(); 
    auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(
        end_ - start_);
    total_ += timeSpan.count();
  }

  inline double get() const {
    return elapsedSeconds();
  }
  
  double elapsedSeconds() const {
    return total_;
  }
  
  friend std::ostream& operator<<(std::ostream& os, const Timer& t) {
      os << t.elapsedSeconds();
      return os;
  }

 private:
  std::chrono::high_resolution_clock::time_point start_;
  std::chrono::high_resolution_clock::time_point end_;
  double total_;
};

class ScopedTimer : public Timer {
 public:
  ScopedTimer() {}

  ~ScopedTimer() {
    stop();
    std::cout << "Elapsed: " << elapsedSeconds() << " s" << std::endl;
  }
};
