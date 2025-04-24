#pragma once

#include <stdexcept>

namespace uclv
{
namespace robot
{

class base_exception : public std::exception
{
  std::string error_str_;

public:
  base_exception(const std::string& error_str) : std::exception(), error_str_(error_str)
  {
  }
  /** Returns a C-style character string describing the general cause
   *  of the current error.  */
  virtual const char* what() const _GLIBCXX_TXN_SAFE_DYN _GLIBCXX_USE_NOEXCEPT override
  {
    return error_str_.c_str();
  }
};

class runtime_error : public base_exception
{
public:
  runtime_error(const std::string& msg = "") : base_exception(msg)
  {
  }
};

class exceeded_joint_limits : public runtime_error
{
public:
  exceeded_joint_limits(const std::string& msg = "") : runtime_error(msg)
  {
  }
};

}  // namespace robot
}  // namespace uclv
