#pragma once

#include <boost/format.hpp>
#include <ros/console.h>

/** \file Logging
 *  Functions for logging throughout Robowflex. Uses the boost::format format string syntax.
 *  See https://www.boost.org/doc/libs/1_66_0/libs/format/doc/format.html for details.
 *  All functions follow a 'printf' style, where there is a format string followed by arguments.
 *  e.g., RBX_ERROR("%s failed! Try %+08d", "test", 10)
 *        RBX_ERROR("%1%: %2%", "argument 1", "argument 2")
 *
 *  There are the following debugging levels available, in descending order of priority:
 *  - FATAL, which also causes the program to terminate.
 *  - ERROR
 *  - WARN
 *  - INFO
 *  - DEBUG, which are not visible unless enabled (e.g., with showUpToDebug())
 *
 *  Currently, all logging is done through rosconsole. It is good practice to use the defined
 *  macros here for abstraction purpose.
 */

namespace moveit_benchmark_suite
{
/** \brief Logging functions.
 */
namespace log
{
/** \brief Recursion base case, return string form of formatted arguments.
 *  \param[in] f Formatter.
 *  \return String of formatted arguments.
 */
std::string formatRecurse(boost::format& f);

/** \brief Recursion base case, return string form of formatted arguments.
 *  \tparam[in] T Type of first argument.
 *  \tparam[in] Args format argument types.
 *  \param[in] f Formatter.
 *  \param[in] t First argument.
 *  \param[in] args Remaining format arguments.
 *  \return String of formatted arguments.
 */
template <class T, class... Args>
std::string formatRecurse(boost::format& f, T&& t, Args&&... args)
{
  return formatRecurse(f % std::forward<T>(t), std::forward<Args>(args)...);
}

/** \brief Recursion base case, return string form of formatted arguments.
 *  \tparam[in] Args format argument types.
 *  \param[in] fmt Format string.
 *  \param[in] args Format arguments.
 *  \return String of formatted arguments.
 */
template <typename... Args>
std::string format(const std::string& fmt, Args&&... args)
{
  boost::format f(fmt);
  return formatRecurse(f, std::forward<Args>(args)...);
}

/** \brief Show all logging messages fatal and above.
 */
void showUpToFatal();

/** \brief Show all logging messages error and above.
 */
void showUpToError();

/** \brief Show all logging messages warning and above.
 */
void showUpToWarning();

/** \brief Show all logging messages info and above.
 */
void showUpToInfo();

/** \brief Show all logging messages debug and above.
 */
void showUpToDebug();
}  // namespace log
}  // namespace moveit_benchmark_suite
