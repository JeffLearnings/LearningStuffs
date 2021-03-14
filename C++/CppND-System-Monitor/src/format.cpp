#include <string>

#include "format.h"

using std::string;

// TODO: Complete this helper function
// INPUT: Long int measuring seconds
// OUTPUT: HH:MM:SS
// REMOVE: [[maybe_unused]] once you define the function
string Format::ElapsedTime(long seconds) {
  std::string answer{""};
  long hours = seconds / 3600;
  seconds %= 3600;
  long minutes = seconds / 60;
  seconds %= 60;

  answer += ((hours / 10 == 0) ? "0" + (std::to_string(hours))
                               : std::to_string(hours));
  answer.append(":");
  answer += ((minutes / 10 == 0) ? "0" + (std::to_string(minutes))
                                 : std::to_string(minutes));
  answer.append(":");
  answer += ((seconds / 10 == 0) ? "0" + (std::to_string(seconds))
                                 : std::to_string(seconds));

  return answer;
}
