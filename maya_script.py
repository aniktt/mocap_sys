import socket
import maya.cmds as cmds
import threading
import os
import time

host = '0.0.0.0'  # Listen on all available interfaces
port = 12345

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((host, port))
s.listen(1)
stop_script = False
save_file = False
bake_animation = False
animation_data = {}

windowID = "JT_Controller_Tool_Set"
if(cmds.window(windowID,q = True, exists = True)):
    cmds.deleteUI(windowID)
    
if(cmds.dockControl("dockWindows",q = True, exists = True)):
    cmds.deleteUI("dockWindows")

window = cmds.window(windowID, title = "MoCap Capture", rtf = True)

windowLayout = cmds.columnLayout()

StartButton = cmds.button(l = "Start", c = "readServer()", w=300)
StopButton = cmds.button(l = "Stop", c = "stopReadServer()", w=300)

cmds.separator(w=300, h=10)
cmds.text(label="Save in file")
cmds.gridLayout(numberOfColumns=2, cellWidth=150)

SaveButton = cmds.button(l = "Save in File", c = "startSaveFile()", w=150)
StopSaveButton = cmds.button(l = "Stop Saving", c = "stopSaveFile()", w=150)

cmds.setParent('..')  # Exit the grid layout

cmds.separator(w=300, h=10)
cmds.text(label="Bake Animation")
cmds.gridLayout(numberOfColumns=2, cellWidth=150)

BakeButton = cmds.button(l="Bake", c="startBakeAnimation()", w=150)
StopBakeButton = cmds.button(l="Stop Bake", c="stopBakeAnimation()", w=150)

cmds.setParent('..')  # Exit the grid layout

cmds.setParent('..')  # Exit the column layout

cmds.showWindow(windowID)


def readServer():
    global stop_script
    stop_script = False
    print(f"Listening for ESP32 connection on {host}:{port}")
    # Start the communication thread
    thread = threading.Thread(target=communication_thread)
    thread.start()


def stopReadServer():
    global stop_script
    stop_script = True


def startSaveFile():
    global save_file
    save_file = True


def stopSaveFile():
    global save_file
    save_file = False


def saveFile(data):
    if save_file == True:
        with open("angles.txt", "a") as file:
            file.write(data + "\n")
        
        
def startBakeAnimation():
    global bake_animation, start_time
    bake_animation = True
    cmds.currentTime(1, edit=True)
    start_time = time.time()  # Record the start time

def stopBakeAnimation():
    global bake_animation
    bake_animation = False
    bakeAnimation()

  
  
def bakeAnimation():
    global animation_data
    for frame, rotations in animation_data.items():
        for obj, rotation in rotations.items():
            cmds.setKeyframe(obj, attribute='rotateX', t=frame, v=rotation[0])
            cmds.setKeyframe(obj, attribute='rotateY', t=frame, v=rotation[1])
            cmds.setKeyframe(obj, attribute='rotateZ', t=frame, v=rotation[2])
    animation_data.clear()
    
    

def communication_thread():
    global stop_script
    try:
        conn, addr = s.accept()
        print('Connected by', addr)
        
        while not stop_script:
            data = conn.recv(1024).decode("utf-8")
            if not data:
                break
            
            # Process the received data and perform actions in Maya accordingly
            print(data)
            values = data.split()
            if len(values) >= 1:  
                #frame = cmds.currentTime(query=True)
                strr = 'GRP_' + values[0]
                rotation = [-float(values[2]), -float(values[1]), float(values[3])]
                
                try:

                    cmds.evalDeferred(lambda: cmds.setAttr(strr  + '.rotateX', -float(values[2])))
                    cmds.evalDeferred(lambda: cmds.setAttr(strr  + '.rotateY', -float(values[1])))
                    cmds.evalDeferred(lambda: cmds.setAttr(strr  + '.rotateZ', float(values[3])))
                    
                    if bake_animation:
                        elapsed_time = time.time() - start_time
                        frame = int(elapsed_time * 24)

                        if frame not in animation_data:
                            animation_data[frame] = {}
                        animation_data[frame][strr] = rotation
                    
                    saveFile(data)
                    
                except Exception as e:
                    print("Error setting attribute:", str(e))
            else:
                print("No data received or insufficient data.")
                                  
            #cmds.refresh()
        conn.close()
    except Exception as e:
        print(f"Exception in communication thread: {e}")




 