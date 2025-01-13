include "mid360_mapping_nanjing.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 6, --3
}

POSE_GRAPH.optimize_every_n_nodes = 10  --20



return options


