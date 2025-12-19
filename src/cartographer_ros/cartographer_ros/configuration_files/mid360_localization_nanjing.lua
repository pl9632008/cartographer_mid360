include "mid360_mapping_nanjing.lua"



-- 在纯定位模式下，最多保留多少个子地图（submap）参与匹配

-- 子地图越多
-- ✅ 可匹配区域更大
-- ❌ 计算量、内存占用更高

-- 子地图越少
-- ✅ 更省算力
-- ❌ 容易在“快速移动 / 大场景 / 回环附近”定位不稳
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 9, 
}

POSE_GRAPH.optimize_every_n_nodes = 20  -- 在线回环：若大于 0，在建图过程中每 N 个节点执行一次回环优化。

POSE_GRAPH.global_sampling_ratio = 0.009 -- 0.003  全局回环时，对 node 的采样比例, 不是拿所有 node 去做全局匹配，而是抽样
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3 -- 0.3  当“已添加的约束 / 潜在约束”的比例低于该数值时，才会添加新的约束。

POSE_GRAPH.constraint_builder.max_constraint_distance = 20 --15 只有当 node 位姿 和 submap 中心的距离 < 这个值，才会尝试 scan matching 建立约束。

POSE_GRAPH.global_constraint_search_after_n_seconds = 10.0 --10 多久没有成功回环，就“放弃局部搜索”，改成全局搜索

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45 -- 90 在创建一个新子地图之前需要插入的激光数据帧数。每个子地图实际会插入两倍数量的数据：前一半用于初始化（不参与匹配）后一半在参与匹配的同时插入

return options

