// Copyright 2026 u5-4
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "bag_contract_probe/analyzer.hpp"

namespace
{

struct CommandLine
{
  std::filesystem::path bag_uri;
  std::filesystem::path report_directory;
  bool show_help{false};
};

void PrintUsage(const char * program)
{
  std::cout << "Usage: " << program <<
    " --bag <rosbag2-directory> --output <report-directory>\n";
}

CommandLine ParseCommandLine(const int argc, char ** argv)
{
  CommandLine options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      options.show_help = true;
    } else if (argument == "--bag" || argument == "--output") {
      if (index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a value");
      }
      const std::filesystem::path value = argv[++index];
      if (argument == "--bag") {
        options.bag_uri = value;
      } else {
        options.report_directory = value;
      }
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (!options.show_help &&
    (options.bag_uri.empty() || options.report_directory.empty()))
  {
    throw std::invalid_argument("--bag and --output are required");
  }
  return options;
}

}  // namespace

int main(const int argc, char ** argv)
{
  try {
    const CommandLine command_line = ParseCommandLine(argc, argv);
    if (command_line.show_help) {
      PrintUsage(argv[0]);
      return 0;
    }
    const auto outcome = bag_contract_probe::AnalyzeBag(
      bag_contract_probe::AnalysisRequest{
      command_line.bag_uri, command_line.report_directory});
    std::cout << "[COMPLETE] Read-only bag analysis completed\n" <<
      "runtime_contract=" <<
      bag_contract_probe::ToString(outcome.runtime_contract_verdict) << '\n' <<
      "trajectory_accuracy=NOT_AUTHORIZED\n" <<
      "flight_authorization=DENIED\n" <<
      "messages_read=" << outcome.messages_read << '\n' <<
      "findings=" << outcome.finding_count << '\n' <<
      "summary=" << outcome.summary_path << '\n';
    return outcome.runtime_contract_verdict ==
           bag_contract_probe::RuntimeContractVerdict::kFail ? 2 : 0;
  } catch (const bag_contract_probe::EvidenceContractError & error) {
    std::cerr << "[FAIL] evidence contract violation: " << error.what() << '\n';
    return 2;
  } catch (const std::exception & error) {
    std::cerr << "[STOP] " << error.what() << '\n';
    PrintUsage(argv[0]);
    return 1;
  }
}
