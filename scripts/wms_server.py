from fastapi import FastAPI, Request
import uvicorn
import requests

app = FastAPI()

@app.post("/pick")
async def pick_item(request: Request):
    data = await request.json()
    print(f"Received pick request: {data}")
    
    # Forward to robot controller
    robot_response = requests.post("http://127.0.0.1:9001/pick", json=data)
    
    return {"message": "Request forwarded", "robot_response": robot_response.json()}

@app.post("/confirmPick")
async def confirm_pick(request: Request):
    data = await request.json()
    print(f"[WMS] Confirmation received: {data}")
    return {"status": "confirmation received", "pickId": data["pickId"]}

if __name__ == "__main__":
    uvicorn.run("wms_server:app", host="127.0.0.1", port=9000, reload=False)
