#include <boost/lexical_cast.hpp>
#include <utility>
#include <boost/variant.hpp>
#include <moveit_benchmark_suite/dataset.h>

using namespace moveit_benchmark_suite;

///
/// PlannerMetric
///

namespace
{
class toMetricStringVisitor : public boost::static_visitor<std::string>
{
public:
  std::string operator()(int value) const
  {
    return std::to_string(value);
  }

  std::string operator()(double value) const
  {
    // double v = boost::get<double>(value);

    // [Bad Pun] No NaNs, Infs, or buts about it.
    return boost::lexical_cast<std::string>(  //
        (std::isfinite(value)) ? value : std::numeric_limits<double>::max());
  }

  std::string operator()(std::size_t value) const
  {
    return std::to_string(value);
  }

  std::string operator()(bool value) const
  {
    return boost::lexical_cast<std::string>(value);
  }

  std::string operator()(const std::string& value) const
  {
    return value;
  }
};

class toMetricDoubleVisitor : public boost::static_visitor<double>
{
public:
  double operator()(int value) const
  {
    return static_cast<double>(value);
  }

  double operator()(double value) const
  {
    return static_cast<double>(value);
  }

  double operator()(std::size_t value) const
  {
    return static_cast<double>(value);
  }

  double operator()(bool value) const
  {
    return static_cast<double>(value);
  }

  double operator()(const std::string& value) const
  {
    return boost::lexical_cast<double>(value);
  }
};
}  // namespace

std::string moveit_benchmark_suite::toMetricString(const Metric& metric)
{
  return boost::apply_visitor(toMetricStringVisitor(), metric);
}

double moveit_benchmark_suite::toMetricDouble(const Metric& metric)
{
  return boost::apply_visitor(toMetricDoubleVisitor(), metric);
}