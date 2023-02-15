/**
 * @file ExecutorObserver.cpp
 *
 * Implementation of class ExecutorObserver.
 */

#include "ExecutorObserver.h"

std::chrono::time_point<std::chrono::steady_clock> ExecutorObserver::_origin;
std::mutex ExecutorObserver::_origin_mutex;
std::atomic_char ExecutorObserver::_instanceId{0};
