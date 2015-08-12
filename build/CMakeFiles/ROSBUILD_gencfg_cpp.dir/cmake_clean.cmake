FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/cloud_to_map/cloud_to_map_nodeConfig.h"
  "../docs/cloud_to_map_nodeConfig.dox"
  "../docs/cloud_to_map_nodeConfig-usage.dox"
  "../src/cloud_to_map/cfg/cloud_to_map_nodeConfig.py"
  "../docs/cloud_to_map_nodeConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
