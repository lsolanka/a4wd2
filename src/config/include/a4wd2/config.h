#include <cxxopts.hpp>
#include <spdlog/spdlog.h>

namespace a4wd2::config
{

/** Extract the ``verbosity`` option and set the globallogging level for the executable.
 */
bool setup_verbosity(const cxxopts::ParseResult& options);

} // namespace a4wd2::config
