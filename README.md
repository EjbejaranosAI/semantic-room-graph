# Semantic Room Graphs from 2D SLAM for Instruction-to-Goal Navigation

A lightweight ROS2 pipeline that transforms 2D occupancy maps into a queryable semantic room graph, enabling instruction-to-goal navigation in real environments.

**Target:** IEEE RO-MAN 2026

---

## Repository Structure

```
semantic-room-graph/           ← clone into ~/your_ws/src/
├── semantic_map_msgs/         # Custom ROS2 messages (CMake)
│   └── msg/                   # 9 message definitions
├── semantic_map_core/         # Core pipeline nodes (Python, 8 nodes)
│   ├── launch/                # Stage A + full launch files
│   └── config/                # core_params.yaml
├── semantic_map_perception/   # Perception nodes (Python, 3 nodes)
│   └── config/                # perception_params.yaml
├── semantic_map_query/        # Query pipeline nodes (Python, 4 nodes)
│   ├── launch/                # Stage B launch file
│   └── config/                # query_params.yaml
├── docs/                      # Design documents
└── README.md
```

## Usage

### 1. Clone into your ROS2 workspace

```bash
cd ~/your_ws/src
git clone git@github.com:EjbejaranosAI/semantic-room-graph.git
```

Your workspace should look like:
```
~/your_ws/src/
├── sfm/                       ← your existing SFM controller
└── semantic-room-graph/
    ├── semantic_map_msgs/
    ├── semantic_map_core/
    ├── semantic_map_perception/
    └── semantic_map_query/
```

### 2. Install dependencies

```bash
# EasyOCR (text detection)
pip install easyocr

# Ollama (VLM + LLM inference)
# Install: https://ollama.com
ollama pull llava       # VLM for room classification
ollama pull llama3      # LLM for semantic query fallback
```

### 3. Build

```bash
cd ~/your_ws

# Step 1: Messages first (all other packages depend on these)
colcon build --packages-select semantic_map_msgs
source install/setup.bash

# Step 2: All Python packages
colcon build --packages-select semantic_map_core semantic_map_perception semantic_map_query
source install/setup.bash
```

### 4. Run Stage A — Build the Semantic Graph

Prerequisites: `/map` published, Nav2 active, camera on `/head_front_camera/rgb/image_raw`.

```bash
ros2 launch semantic_map_core stage_a_graph_construction.launch.py
```

The robot will:
1. Ingest the map and segment regions
2. Navigate to each region (via Nav2)
3. Capture images, run OCR + VLM
4. Fuse labels and save graph to `/tmp/semantic_map/semantic_graph.json`

### 5. Run Stage B — Instruction to Goal

```bash
# Launch query pipeline
ros2 launch semantic_map_query stage_b_query.launch.py

# Load saved graph
ros2 service call /semantic_map/load_graph std_srvs/srv/Trigger

# Send a query
ros2 topic pub --once /semantic_map/user_query std_msgs/msg/String "{data: 'go to office 203'}"
```

### 6. Full pipeline (both stages)

```bash
ros2 launch semantic_map_core full_pipeline.launch.py
```

---

## Pipeline Flow

### Stage A: Semantic Graph Construction

```
/map → map_ingestor → region_segmentation → topo_graph_builder
                                                    │
                                             pose_generator
                                                    │
                                          exploration_planner
                                                    │
                                         navigator_executor → Nav2
                                                    │
                                        perception_collector
                                            ┌───────┴───────┐
                                        ocr_node       vlm_labeler
                                            └───────┬───────┘
                                         semantic_fusion
                                                │
                                          graph_store → JSON
```

### Stage B: Instruction-to-Goal

```
"Go to office 203" → instruction_parser → semantic_retriever
                                                  │
                                           goal_resolver
                                                  │
                                          task_navigator → Nav2
```

---

## Configuration

Key parameters to tune:

| Parameter | File | Default | Description |
|-----------|------|---------|-------------|
| `min_region_area` | `semantic_map_core/config/core_params.yaml` | 500 | Min region size in pixels |
| `vlm_confidence_threshold` | `semantic_map_core/config/core_params.yaml` | 0.60 | Min VLM score to label |
| `camera_topic` | `semantic_map_perception/config/perception_params.yaml` | `/head_front_camera/rgb/image_raw` | RGB camera topic |
| `model_name` | `semantic_map_perception/config/perception_params.yaml` | `llava` | VLM model |
| `llm_model` | `semantic_map_query/config/query_params.yaml` | `llama3` | LLM for semantic fallback |
| `similarity_threshold` | `semantic_map_query/config/query_params.yaml` | 0.50 | Min string match score |

---

## Debug

```bash
ros2 topic list | grep semantic_map
ros2 topic echo /semantic_map/semantic_graph
ros2 topic echo /semantic_map/ocr_result
ros2 topic echo /semantic_map/vlm_result
cat /tmp/semantic_map/semantic_graph.json | python3 -m json.tool
```

---

## License

MIT
