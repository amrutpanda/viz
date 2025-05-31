/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#define _STRINGIZE(x) #x
#define STRINGIZE(x) _STRINGIZE(x)

namespace mcx::meril_robot {

const char* version() { return STRINGIZE(MOTORCORTEX_ROBOT_VERSION); }

} // namespace mcx::meril_robot