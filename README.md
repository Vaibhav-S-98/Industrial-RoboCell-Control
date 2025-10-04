# Bin Picking Cell Control System – ROS2 + FastAPI Project

This project simulates a full industrial-style bin-picking robot cell using ROS 2 (Humble), FastAPI, and a simulated WMS (Warehouse Management System). It includes barcode scanning, emergency stop, door safety, and light signaling.

---

1. Prerequisites

- OS: Ubuntu 22.04
- ROS 2 Humble installed
- Python 3.10+

---

2. Project Setup (Folder Structure)

```bash
bin_picking_ws/
├── src/
│   ├── scanner_node/
│   ├── door_node/
│   ├── emergency_node/
│   ├── stack_light_node/
├── scripts/
│   ├── cell_controller_node.py
│   ├── wms_server.py
```

3. Workspace Creation & Dependencies
```bash
# Create workspace
mkdir -p ~/bin_picking_ws/src
cd ~/bin_picking_ws

# Create a Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python libraries
pip install fastapi uvicorn requests
```
4. Creating ROS2 Packages (Optional)

If you're starting fresh, create packages:
```bash
cd ~/bin_picking_ws/src

ros2 pkg create --build-type ament_python door_node
ros2 pkg create --build-type ament_python emergency_node
ros2 pkg create --build-type ament_python scanner_node
ros2 pkg create --build-type ament_python stack_light_node
#Put your node.py files inside each package under:
#your_package/your_package/your_node.py
```

5. Building ROS2 Packages
```bash
cd ~/bin_picking_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
You must do this after every code change.
```
6. What Each Node Does

- scanner_node: Publishes a barcode string every second

- door_node: Publishes door status and handles a service to return state

- emergency_node: Publishes emergency stop status and handles service

- stack_light_node: Subscribes to door/emergency, sets light status

- cell_controller_node.py: Core logic to check safety, trigger barcode, and confirm pick

- wms_server.py: Simulates WMS, sends pick request and receives confirmation

7. Running the System (Start ROS Nodes)
```bash
Terminal 1 – Start door node

    source /opt/ros/humble/setup.bash
    source ~/bin_picking_ws/install/setup.bash
    ros2 run door_node door_node
      
Terminal 2 – start emergency node

    source ~/bin_picking_ws/venv/bin/activate
    python3 scripts/cell_controller_node.py
    ros2 run emergency_node emergency_node

Terminal 3 – Start scanner node
    source /opt/ros/humble/setup.bash
    source ~/bin_picking_ws/install/setup.bash
    ros2 run scanner_node scanner_node

Terminal 4 – start stck light node

    source /opt/ros/humble/setup.bash
    source ~/bin_picking_ws/install/setup.bash
    ros2 run stack_light_node stack_light_node

Terminal 5 – Start WMS Server

    source ~/bin_picking_ws/venv/bin/activate
    python3 scripts/wms_server.py

Terminal 6 – Start cell controller node

    source ~/bin_picking_ws/venv/bin/activate
    python3 scripts/cell_controller_node.py
```
8. Testing the Flow (new terminal)

Send a pick request:
```bash
  curl -X POST http://127.0.0.1:9000/pick \
    -H "Content-Type: application/json" \
    -d '{"pickId": 101, "quantity": 2}'

You should see logs in:

  cell_controller_node.py: ROS calls, barcode, confirmation sent
    [FASTAPI] /pick route hit
    [ROS CALL] Starting call to /get_door_state 
    [ROS CALL] Response received from /get_door_state
    [ROS CALL] Response received from /get_barcode
    [CELL] Confirmation sent: 200 - {...}

  wms_server.py: pick request received, confirmation received
    [WMS] Received pick request: {...}
    [WMS] Confirmation received: {
      "pickId": 123,
      "pickSuccessful": true,
      "errorMessage": null,
      "itemBarcode": 82736
```

10. Troubleshooting

-- Node object is not callable:
    Don't name service handler def handle(). Use def handle_request()

-- generator already executing
    Avoid blocking inside FastAPI. Use ThreadPoolExecutor correctly