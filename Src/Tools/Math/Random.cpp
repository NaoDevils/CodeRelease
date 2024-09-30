/**
 * @file Math/Random.cpp
 * This class defines some random number functions.
 * @author <a href="mailto:aaorn.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#include "Random.h"

thread_local std::default_random_engine Random::generator(std::random_device{}());
