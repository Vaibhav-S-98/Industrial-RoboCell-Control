import asyncio
import rclpy
from rclpy.node import Node
import requests
from fastapi import FastAPI, Request
import uvicorn
import threading
import time
from example_interfaces.srv import Trigger
import concurrent.futures
from concurrent.futures import ThreadPoolExecutor
import traceback
from rclpy.executors import SingleThreadedExecutor
import os
print("[BOOT] Running in process:", os.getpid())

app = FastAPI()
ros_node: Node = None
ros_ready = threading.Event()

# Reuse this globally to avoid creating threads every request
thread_pool = ThreadPoolExecutor()

@app.post("/pick")
async def receive_pick_request(request: Request):
    print("[FASTAPI] /pick route hit")
    data = await request.json()
    pick_id = data["pickId"]

    # Wrap both ROS service calls AND confirmation request
    def ros_and_confirm_logic():
        door_resp = ros_node.call_service('/get_door_state')
        emergency_resp = ros_node.call_service('/get_emergency_state')

        if not door_resp or not door_resp.success or emergency_resp.success:
            result = {
                "pickId": pick_id,
                "pickSuccessful": False,
                "errorMessage": "Safety Condition Failed",
                "itemBarcode": 0
            }
        else:
            barcode_resp = ros_node.call_service('/get_barcode')
            barcode = int(barcode_resp.message) if barcode_resp else 0
            result = {
                "pickId": pick_id,
                "pickSuccessful": True,
                "errorMessage": None,
                "itemBarcode": barcode
            }

        try:
            response = requests.post("http://127.0.0.1:9000/confirmPick", json=result, timeout=10)
        except Exception:
            pass

        return result

    loop = asyncio.get_event_loop()
    result = await loop.run_in_executor(thread_pool, ros_and_confirm_logic)

    return {"status": "processed"}




'''class CellNode(Node):
    def __init__(self):
        super().__init__('cell_controller_node')

    def call_service(self, service_name):
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"{service_name} not available")
            return None

        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() '''
class CellNode(Node):
    def __init__(self):
        super().__init__('cell_controller_node')
        self._thread_pool = concurrent.futures.ThreadPoolExecutor()

    def call_service(self, service_name):
        # Offload blocking service call to avoid FastAPI async issues
        future = self._thread_pool.submit(self._blocking_service_call, service_name)
        return future.result()

    def _blocking_service_call(self, service_name):
        print(f"[ROS CALL] Starting call to {service_name}")
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"{service_name} not available")
            return None

        request = Trigger.Request()
        future = client.call_async(request)

        executor = SingleThreadedExecutor()
        executor.add_node(self)
        print(f"[ROS CALL] Waiting for {service_name} response")
        executor.spin_until_future_complete(future)
        print(f"[ROS CALL] Response received from {service_name}")
        return future.result()


#def run_server():
#    print("[SERVER] Starting FastAPI")
#    uvicorn.run("cell_controller_node:app", host="0.0.0.0", port=8081, reload=False)

def run_server():
    print("[SERVER] Starting FastAPI with app object")
    uvicorn.run(app, host="127.0.0.1", port=9001, reload=False)

def main(args=None):
    global ros_node
    print("[MAIN] Initializing ROS2")
    rclpy.init(args=args)

    print("[MAIN] Creating ROS node...")
    ros_node = CellNode()
    print("[MAIN] ROS node created:", ros_node.get_name())
    ros_ready.set()

    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()

    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("[BOOT] Starting main()")
    main()
