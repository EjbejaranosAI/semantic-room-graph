# Semantic Room Graphs from 2D SLAM for Instruction-to-Goal Navigation

A lightweight ROS2 pipeline that transforms 2D occupancy maps into a queryable semantic room graph, enabling instruction-to-goal navigation in real environments.

**Target:** IEEE RO-MAN 2026

---

## Project Structure

```
semantic_room_graph_ws/
├── docs/                          # Design documents
│   ├── semantic_room_graphs_final.docx
│   └── ros2_architecture_messages_design.docx
├── src/
│   ├── semantic_map_msgs/         # Custom ROS2 messages (CMake)
│   │   └── msg/                   # 9 message definitions
│   ├── semantic_map_core/         # Core pipeline nodes (Python)
│   │   ├── semantic_map_core/     # 8 nodes
│   │   ├── launch/                # Stage A + full launch files
│   │   └── config/                # core_params.yaml
│   ├── semantic_map_perception/   # Perception nodes (Python)
│   │   ├── semantic_map_perception/  # 3 nodes (collector, OCR, VLM)
│   │   ├── launch/
│   │   └── config/                # perception_params.yaml
│   ├── semantic_map_query/        # Query pipeline nodes (Python)
│   │   ├── semantic_map_query/    # 4 nodes
│   │   ├── launch/                # Stage B launch file
│   │   └── config/                # query_params.yaml
│   └── sfm/                       # Social Force Model Nav2 controller (C++)
└── README.md
```

---

## How the Pipeline Executes

### Stage A: Build the Semantic Room Graph

This is the **offline** stage. The robot explores the environment and builds a labeled graph.

```
Step 1: /map (from SLAM)
    │
    ▼
Step 2: map_ingestor_node
    │  Subscribes to /map (OccupancyGrid)
    │  Publishes binary free-space image + map metadata
    │
    ▼
Step 3: region_segmentation_node
    │  Takes the binary image
    │  Segments free space into regions (watershed + morphology)
    │  Publishes regions_raw (SemanticGraph with no edges yet)
    │
    ▼
Step 4: topological_graph_builder_node
    │  Detects region adjacency (shared boundaries)
    │  Creates edges between neighboring regions
    │  Publishes topological_graph
    │
    ▼
Step 5: pose_generator_node
    │  Generates entry_pose and inspection_pose per region
    │  Places poses in navigable free space (max distance from walls)
    │  Publishes graph_with_poses
    │
    ▼
Step 6: exploration_planner_node
    │  Computes visit order (nearest-neighbor heuristic)
    │  Publishes ordered list of region IDs
    │
    ▼
Step 7: navigator_executor_node
    │  Sends each region's entry_pose to Nav2 (NavigateToPose action)
    │  Uses SFM controller for human-aware navigation
    │  On arrival, publishes region_visit_event
    │
    ▼
Step 8: perception_collector_node
    │  Captures RGB image from camera when robot reaches region
    │  Publishes RegionObservation
    │
    ├──────────────────┐
    ▼                  ▼
Step 9a: ocr_node     Step 9b: vlm_labeler_node
    │  EasyOCR            │  VLM (Ollama/OpenAI)
    │  Detects text        │  Classifies room type
    │  (door numbers,      │  (office, kitchen, etc.)
    │   names, signs)      │
    ▼                  ▼
    └──────┬───────────┘
           ▼
Step 10: semantic_fusion_node
    │  Fuses OCR + VLM results
    │  OCR prioritized for explicit IDs (e.g. "Office 203")
    │  VLM provides room type if confidence > 0.60
    │  Updates region status → labeled
    │
    ▼
Step 11: graph_store_node
    │  Saves SemanticGraph as JSON to disk
    │  Auto-saves on every update
    │  Provides load/save services
    │
    ▼
    📁 /tmp/semantic_map/semantic_graph.json  ← DONE
```

### Stage B: Instruction-to-Goal Navigation

This is the **online** stage. The user gives a text instruction and the robot navigates there.

```
Step 1: User says "Go to office 203"
    │  (from your ASR pipeline → publishes to /semantic_map/user_query)
    │
    ▼
Step 2: instruction_parser_node
    │  Extracts numeric cues: has_numeric=true, ref=203
    │  Publishes QueryRequest
    │
    ▼
Step 3: semantic_retriever_node
    │  Stage 1: Deterministic lookup
    │    → Searches OCR text for "203" → exact match
    │    → Searches semantic labels for "office" → fuzzy match
    │  Stage 2: Semantic fallback (if no match)
    │    → Sends query to LLM with known graph labels
    │    → LLM returns ranked candidates
    │    → Projects onto known categories
    │  Publishes QueryResult with ranked candidates
    │
    ▼
Step 4: goal_resolver_node
    │  Takes the best candidate region
    │  Resolves it to a PoseStamped (entry_pose of that region)
    │  Publishes /semantic_map/final_goal
    │
    ▼
Step 5: task_navigator_node
    │  Sends goal to Nav2 NavigateToPose action
    │  SFM controller handles the actual navigation
    │  Reports success/failure on /semantic_map/task_result
    │
    ▼
    🤖 Robot arrives at "Office 203"
```

---

## Topic Data Flow

```
/map ─────────────────────► map_ingestor_node
                                │
             /semantic_map/grid_image ──────► region_segmentation_node
             /semantic_map/map_metadata ─────┘        │
                                        /semantic_map/regions_raw
                                                      │
             /semantic_map/label_image ──────► topological_graph_builder_node
                                                      │
                                     /semantic_map/topological_graph
                                                      │
             /semantic_map/grid_image ──────► pose_generator_node
                                                      │
                                      /semantic_map/graph_with_poses
                                            │         │
                                            ▼         ▼
                              exploration_planner  semantic_fusion_node
                                      │
                          /semantic_map/exploration_plan
                                      │
                              navigator_executor_node ──► Nav2 (SFM)
                                      │
                          /semantic_map/region_visit_event
                                      │
                              perception_collector_node
                                      │
                          /semantic_map/region_observation
                                    │   │
                              ocr_node  vlm_labeler_node
                                │           │
                    /semantic_map/ocr_result  /semantic_map/vlm_result
                                └─────┬─────┘
                              semantic_fusion_node
                                      │
                          /semantic_map/semantic_graph
                                      │
                              graph_store_node ──► JSON file
                                      │
              ┌───────────────────────┘
              ▼
/semantic_map/user_query ──► instruction_parser_node
                                      │
                          /semantic_map/query_request
                                      │
                              semantic_retriever_node
                                      │
                          /semantic_map/query_candidates
                                      │
                              goal_resolver_node
                                      │
                          /semantic_map/final_goal
                                      │
                              task_navigator_node ──► Nav2 (SFM)
                                      │
                          /semantic_map/task_result
```

---

## Build & Run

### 1. Build (on the robot or dev machine with ROS2)

```bash
cd semantic_room_graph_ws

# Step 1: Build messages first (other packages depend on these)
colcon build --packages-select semantic_map_msgs
source install/setup.bash

# Step 2: Build the SFM controller (C++)
colcon build --packages-select sfm
source install/setup.bash

# Step 3: Build the Python packages
colcon build --packages-select semantic_map_core semantic_map_perception semantic_map_query
source install/setup.bash
```

### 2. Run Stage A (build the semantic graph)

Prerequisites: SLAM running and publishing `/map`, camera on `/camera/rgb/image_raw`, Nav2 active.

```bash
# Terminal 1: Launch Stage A
ros2 launch semantic_map_core stage_a_graph_construction.launch.py

# The pipeline will automatically:
# 1. Ingest the map
# 2. Segment regions
# 3. Build the graph
# 4. Generate poses
# 5. Plan exploration
# 6. Navigate to each region
# 7. Capture images
# 8. Run OCR + VLM
# 9. Fuse labels
# 10. Save graph to /tmp/semantic_map/semantic_graph.json
```

### 3. Run Stage B (query the graph)

Prerequisites: A built graph (JSON) or Stage A complete. Nav2 active.

```bash
# Terminal 1: Launch Stage B
ros2 launch semantic_map_query stage_b_query.launch.py

# Terminal 2: Load the saved graph
ros2 service call /semantic_map/load_graph std_srvs/srv/Trigger

# Terminal 3: Send a query (from your ASR pipeline or manually)
ros2 topic pub --once /semantic_map/user_query std_msgs/msg/String \
  "{data: 'go to office 203'}"

# Or run both stages at once:
ros2 launch semantic_map_core full_pipeline.launch.py
```

### 4. Integration with your ASR pipeline

Your existing ASR project should publish the recognized text to:

```
/semantic_map/user_query  (std_msgs/msg/String)
```

The task result (success/failure) is published to:

```
/semantic_map/task_result  (std_msgs/msg/String, JSON format)
```

---

## External Dependencies

| Dependency | Purpose | Install |
|-----------|---------|---------|
| ROS2 Humble+ | Framework | [docs.ros.org](https://docs.ros.org) |
| Nav2 | Navigation stack | `apt install ros-$ROS_DISTRO-navigation2` |
| OpenCV | Image segmentation | Bundled with ROS2 |
| cv_bridge | ROS2 ↔ OpenCV | `apt install ros-$ROS_DISTRO-cv-bridge` |
| EasyOCR | Text detection | `pip install easyocr` |
| Ollama | Local VLM inference | [ollama.com](https://ollama.com) |

---

## Configuration

All parameters are in YAML files under each package's `config/` directory.
Key parameters to tune for your environment:

| Parameter | File | Default | Description |
|-----------|------|---------|-------------|
| `min_region_area` | core_params.yaml | 500 | Min region size in pixels |
| `vlm_confidence_threshold` | core_params.yaml | 0.60 | Min VLM score to label |
| `model_name` | perception_params.yaml | llava | VLM model for room classification |
| `llm_model` | query_params.yaml | llama3 | LLM for semantic fallback |
| `similarity_threshold` | query_params.yaml | 0.50 | Min string similarity for match |

---

## License

MIT
