#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "apps/gyro_alignment.hpp"

namespace phad::apps
{

  struct GyroAlignmentInputIdentity
  {
    std::string basename;
    std::string expected_sha256;
  };

  struct GyroAlignmentProtocolIdentity
  {
    std::string design_commit;
    std::string design_tree;
    std::string design_blob;
    std::string design_sha256;
  };

  struct GyroAlignmentAnalyzerIdentity
  {
    std::string source_commit;
    std::string source_tree;
    std::string build_type;
    std::string compiler;
    std::string compiler_version;
  };

  struct GyroAlignmentProvenance
  {
    std::string q1_source_run;
    std::string q1_source_commit;
    std::string q1_git_tree_object;
    std::string q1_git_ls_tree_sha256;
    std::string q1_meta_sha256;
    std::string q1_config_hash;
    std::string q1_input_manifest_v2_sha256;
    std::string q2_commit;
    std::string q2_tree;
  };

  struct GyroAlignmentProtocolDescriptor
  {
    std::string                             protocol_id;
    std::vector<GyroAlignmentInputIdentity> inputs;
    GyroAlignmentProtocolIdentity           protocol_identity;
    GyroAlignmentAnalyzerIdentity           analyzer_identity;
    GyroAlignmentProvenance                 provenance;
  };

  [[nodiscard]] GyroAlignmentProtocolDescriptor
  makeGyroAlignmentV1ProtocolDescriptor();

  [[nodiscard]] GyroAlignmentResult runGyroAlignment(
      GyroAlignmentProtocolDescriptor descriptor,
      const std::filesystem::path&    q1_dir,
      const std::filesystem::path&    out_dir );

}  // namespace phad::apps
